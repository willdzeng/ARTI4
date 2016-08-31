#include <arti_hardware/arti_hardware.h>

namespace arti_hardware
{

/**
 * @brief      Constructs the object.
 *
 * @param[in]  nh          The public node handle
 * @param[in]  private_nh  The private node handle
 */
ArtiHardware::ArtiHardware(ros::NodeHandle nh, ros::NodeHandle private_nh): nh_(nh), temp_cutoff_(false)
{
	private_nh.param("port", port_, std::string("/dev/ttyACM0"));
	private_nh.param("body_width", body_width_, 1.0);
	private_nh.param("baud_rate", baud_rate_, 9600);
	private_nh.param("serial_time_out", serial_time_out_, 100);
	private_nh.param("control_rate", control_rate_, 30.0);
	private_nh.param("odom_rate", odom_rate_, 50.0);
	private_nh.param("odom_window", odom_window_, 5);
	private_nh.param("cmd_time_out", cmd_time_out_, 500.0);
	private_nh.param("wheel_multiplier", wheel_multiplier_, 0.5);
	private_nh.param("maximum_vel", maximum_vel_, 1.0);
	private_nh.param("odom_bias", odom_bias_, 1.0);
	private_nh.param("ultra_dist_multipiler", ultra_dist_multipiler_, 1.0);
	private_nh.param("temp_multipiler", temp_multipiler_, 1.0);
	private_nh.param("temp_cutoff_value", temp_cutoff_value_, 65.0);
	private_nh.param("flip_lr", flip_lr_, false);
	private_nh.param("publish_tf", publish_tf_, false);
	private_nh.param("base_frame_id", base_frame_id_, std::string("base_link"));

	ROS_INFO("Arti Hardware got port %s", port_.c_str());
	ROS_INFO("Set Serial Timeout %d ms", serial_time_out_);
	ROS_INFO("Baud Rate %d", baud_rate_);
	ROS_INFO("Control Rate %f", control_rate_);
	ROS_INFO("Command Time out is %f ms", cmd_time_out_);
	cmd_time_out_ = cmd_time_out_/1000;

	serial::Timeout to = serial::Timeout::simpleTimeout(serial_time_out_);
	serial_ = new serial::Serial(port_, baud_rate_, to, serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none);

	diff_odom_pub_ = nh.advertise<arti_msgs::DiffOdom>("diff_odom", 1);
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
	ultra_pub_ = nh.advertise<arti_msgs::Ultrasound>("ultrasound", 1);
	temp_pub_ = nh.advertise<arti_msgs::Temperature>("temperature", 1);

	cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ArtiHardware::cmdVelCallback, this, _1));
	diff_cmd_sub_ = nh.subscribe<arti_msgs::DiffCmd>("diff_cmd_vel", 1, boost::bind(&ArtiHardware::diffCmdCallback, this, _1));

	tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(nh_, "/tf", 100));
	tf_odom_pub_->msg_.transforms.resize(1);
	tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
	tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
	tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";

	ros::Duration(2).sleep();

	if (!serial_->isOpen()) {
		try
		{
			serial_->open();
			ROS_INFO("Connection established on: %s", port_.c_str());
		}
		catch (std::runtime_error ex)
		{
			ROS_FATAL("Serial port open failed: %s (%s)", ex.what(), port_.c_str());
			ros::shutdown();
		}

	}

	// sensorLoop();
	odom_thread_ = new boost::thread(boost::bind(&ArtiHardware::sensorLoop, this));
	controlLoop();

}

ArtiHardware::~ArtiHardware()
{
	odom_thread_->interrupt();
	odom_thread_->join();
	try
	{
		sendMotorCmd(0, 0);
		serial_->close();
		ROS_INFO("Serial port shutting down");
	}
	catch (std::exception ex) {};
	delete serial_;
	delete odom_thread_;
}

/**
 * @brief      the main control loop of a the hardware
 */
void ArtiHardware::controlLoop()
{
	ros::Rate r(control_rate_);
	while (nh_.ok()) {
		ros::spinOnce();
		// if it's been a long time since recelve the command, set command to zero
		double inter_time = (ros::Time::now() - cmd_time_).toSec();
		if ( inter_time > cmd_time_out_ ) {
			cmd_left_ = 0;
			cmd_right_ = 0;
			// std::cout << "inter time: " << inter_time << std::endl;
			cmd_time_ = ros::Time::now();
		}
		if (temp_cutoff_ == 1) {
			cmd_left_ = 0;
			cmd_right_ = 0;
			cmd_time_ = ros::Time::now();
		}
		sendMotorCmd(cmd_left_, cmd_right_);

		r.sleep();
	}
}

/**
 * @brief      publish odometry TF
 */
void ArtiHardware::publishOdomTF()
{
	geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
	odom_frame.header.stamp = ros::Time::now();
	odom_frame.transform.translation.x = px_;
	odom_frame.transform.translation.y = py_;
	odom_frame.transform.rotation = tf::createQuaternionMsgFromYaw(theta_);
	tf_odom_pub_->unlockAndPublish();
}

/**
 * @brief      The main sensor loop
 */
void ArtiHardware::sensorLoop()
{
	ROS_INFO_ONCE("Start to publish odom");
	ros::Rate r(odom_rate_);
	std::string data_str;
	std::string type_str;
	std::string tmp_str, use_str;
	std::vector<int> ultra;
	std::vector<int> odom;
	std::vector<double> temp;
	while (nh_.ok()) {

		if (serial_->available()) {
			try
			{
				tmp_str = serial_->readline(50, "\r");
				use_str = serial_->readline(50, "\n");
				if (use_str[0] == '$') {
					// std::cout << "get some data\n";
					// there is no way that the useful data would be smaller than 10;
					if (use_str.size() < 10) {
						continue;
					}
					type_str = use_str.substr(1, 4);
					data_str = use_str.substr(5, use_str.size() - 5);
					// std::cout << "type_str: " << type_str << std::endl;
					// std::cout << "data_str: " << data_str;
					if (type_str == "ODOM") {
						parseDataStr(data_str, odom);
						processOdom(odom);
					} else if (type_str == "ULTR") {
						parseDataStr(data_str, ultra);
						publishUltrasound(ultra);
					} else if (type_str == "TEMP") {
						parseDataStr(data_str, temp);
						tempCheck(temp);
						publishTemperature(temp);
					}
				}
			}
			catch (serial::SerialException ex) //No data received
			{
				ROS_WARN("Serial read exception: %s", ex.what());
				// continue;
			}
			catch (std::runtime_error ex)
			{
				ROS_WARN("Serial read exception: %s", ex.what());
				// continue;
			}
		}

		if (publish_tf_) {
			publishOdomTF();
		}

		data_str = "";
		type_str = "";
		use_str = "";
		tmp_str = "";
		odom.clear();
		ultra.clear();
		temp.clear();
		r.sleep();
	}
}

/**
 * @brief      check if the temperature reaches the cutoff value
 *
 * @param[in]  temp  The temporary
 */
void ArtiHardware::tempCheck(const std::vector<double>& temp)
{
	for (int i = 0; i < temp.size(); i++) {
		if (temp[i] > temp_cutoff_value_) {
			ROS_WARN("Temp %d reached %f higher than the cut off value %f, Robot is too hot, Shutting Down the Robot", i, temp[i], temp_cutoff_value_);
			temp_cutoff_ = true;
			return;
		}
	}
	temp_cutoff_ = false;
	return;
}

/**
 * @brief      publish temperature value
 *
 * @param[in]  temp  The temporary
 */
void ArtiHardware::publishTemperature(const std::vector<double> temp)
{
	ROS_INFO_ONCE("Start publish temperature information");
	// std::vector<double> temp;
	arti_msgs::Temperature temp_msg;
	temp_msg.header.stamp = ros::Time::now();
	for (int i = 0; i < temp.size(); i++) {
		temp_msg.value.push_back(temp[i] * temp_multipiler_);
	}
	temp_pub_.publish(temp_msg);
}

/**
 * @brief      print the odometry
 *
 * @param[in]  odom  The odom
 */
void ArtiHardware::printOdom(const arti_msgs::DiffOdom& odom)
{
	std::cout << "left travel: " << odom.left_travel << " right travel: " <<
	          odom.right_travel << " left speed: " << odom.left_speed << " right speed:" << odom.right_speed
	          << std::endl;
}

/**
 * @brief      print the vector
 *
 * @param[in]  v     vector
 *
 * @tparam     T     template class
 */
template<class T>
void ArtiHardware::printVector(const T& v)
{
	for (int i = 0; i < v.size(); i++) {
		std::cout << v[i] << " " ;
	}
	std::cout << std::endl;
}

/**
 * @brief      Process odom data, filtering using window, and publish odometry
 *
 * @param[in]  odom  The odom
 */
void ArtiHardware::processOdom(const std::vector<int>& odom) {
	if (odom.size() != 2) {
		return;
	}
	// flip left right value, if flip_lr_ is ture;
	int left = 0;
	int right = 0;
	if (flip_lr_) {
		left = odom[1];
		right = odom[0];
	} else {
		left = odom[0];
		right = odom[1];
	}

	arti_msgs::DiffOdom diff_odom;
	diff_odom.left_travel = left * wheel_multiplier_ * odom_bias_;
	diff_odom.right_travel = right * wheel_multiplier_;
	diff_odom.header.stamp = ros::Time::now();
	double dl, dr;
	double dvx, dwz;
	double dt = 0;
	// if there is not enough data in the stack add it to the queue
	if (diff_odom_queue_.size() == odom_window_) {
		dt = (diff_odom.header.stamp - diff_odom_queue_.front().header.stamp).toSec();
		if (dt < 0.00001) {
			return;
		}
		// std::cout << dt << std::endl;
		dl = diff_odom.left_travel - diff_odom_old_.left_travel;
		dr = diff_odom.right_travel - diff_odom_old_.right_travel;
		diff_odom.left_speed =  (diff_odom.left_travel - diff_odom_queue_.front().left_travel) / dt;
		diff_odom.right_speed = (diff_odom.right_travel - diff_odom_queue_.front().right_travel) / dt;
		diff_odom_queue_.pop();
	}
	vl_ = diff_odom.left_speed;
	vr_ = diff_odom.right_speed;
	diff_odom_old_ = diff_odom;
	diff_odom_queue_.push(diff_odom);
	// std::cout << diff_odom_queue_.size() << std::endl;
	diff_odom_pub_.publish(diff_odom);
	// pose estiamtion and publish the odometry
	LRtoDiff(dl, dr, dvx, dwz);
	// std::cout << dl << " " << dr << std::endl;
	LRtoDiff(vl_, vr_, vx_, wz_);
	integrateExact(dvx, dwz);
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = "odom";
	odom_msg.pose.pose.position.x = px_;
	odom_msg.pose.pose.position.y = py_;
	odom_msg.pose.pose.orientation.z = theta_;
	odom_msg.pose.pose.orientation.w = 1;
	odom_msg.twist.twist.linear.x = vx_;
	odom_msg.twist.twist.angular.z = wz_;
	odom_pub_.publish(odom_msg);

}

/**
 * @brief      publish Ultrasound msges
 *
 * @param[in]  ultra  The ultra
 */
void ArtiHardware::publishUltrasound(const std::vector<int>& ultra)
{
	ROS_INFO_ONCE("Start publish ultrasound information");
	std::vector<double> dist;
	arti_msgs::Ultrasound ultra_msg;
	ultra_msg.header.stamp = ros::Time::now();
	for (int i = 0; i < ultra.size(); i++) {
		ultra_msg.distance.push_back(ultra[i] * ultra_dist_multipiler_);
	}
	ultra_pub_.publish(ultra_msg);
}

/**
 * @brief      parse the odom string
 *
 * @param[in]  str    The string
 * @param      left   The left
 * @param      right  The right
 *
 * @return     { description_of_the_return_value }
 */
bool ArtiHardware::parseOdomStr(const std::string& str, int& left, int& right)
{
	std::vector<int> ids;
	for (int i = 0; i < str.length(); i++) {
		if (str[i] == ',') {
			ids.push_back(i);
		}
	}

	if (ids.size() < 2) {
		return false;
	} else {
		std::string left_str = str.substr(0, ids[0]);
		std::string right_str = str.substr(ids[0] + 1, ids[1] - ids[0] - 1);
		// use boost instead of stoi is more fast and stable
		// std::cout<<str;
		try
		{
			left = boost::lexical_cast<int, std::string>(left_str);
			right = boost::lexical_cast<int, std::string>(right_str);
		}
		catch (boost::bad_lexical_cast ex)
		{
			return false;
		}
		return true;
	}
}


/**
 * @brief      parse the data string
 *
 * @param[in]  str   The string
 * @param[in]  data  The data
 *
 * @return     { success or not }
 */
template<class dataType>
bool ArtiHardware::parseDataStr(const std::string& str, std::vector<dataType>& data_vector)
{
	std::vector<int> ids;
	for (int i = 0; i < str.length(); i++) {
		if (str[i] == ',') {
			ids.push_back(i);
		}
	}

	if (ids.size() < 2) {
		return false;
	}

	for (int i = 0; i < (ids.size() - 1); i++) {
		try {
			// use boost to cast the data
			dataType data = boost::lexical_cast<dataType, std::string>(str.substr(ids[i] + 1, ids[i + 1] - ids[i] - 1));
			// add it to data vector
			data_vector.push_back(data);
		}
		catch (boost::bad_lexical_cast ex) {
			return false;
		}
	}
	return true;
}

/**
 * @brief      { Send the motor command }
 *
 * @param[in]  left   The left command
 * @param[in]  right  The right command
 */
void ArtiHardware::sendMotorCmd(const double& left, const double& right)
{
	if (!serial_->isOpen())
	{
		ROS_ERROR("Serial port not available");
		return;
	}
	if (flip_lr_) {
		sprintf(cmd_, "\r$MOTO,%d,%d,\n", (int) (left * 127), (int) (right * 127));
	} else {

		sprintf(cmd_, "\r$MOTO,%d,%d,\n", (int) (right * 127), (int) (left * 127));
	}
	try
	{

		serial_->write(cmd_); //Send query
		if (left != 0.0 && right != 0.0) {
			std::cout << cmd_;
		}
	}
	catch (serial::SerialException ex)
	{
		ROS_WARN("No response to: \"%s\"", cmd_);
		return;
	}
	catch (std::runtime_error ex)
	{
		ROS_WARN("Exception while sending data, %s", ex.what());
		return;
	}
	return;

}

/**
 * @brief      callback function for subscribe command velcoty
 *
 * @param[in]  msg   The message
 */
void ArtiHardware::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO_ONCE("Arti Hardware Get Command");
	diffToLR(msg->linear.x, msg->angular.z, cmd_left_, cmd_right_);
	thresholdVelocity();
	cmd_time_ = ros::Time::now();
}

/**
 * @brief      callback function to subscribe the differetial command
 *
 * @param[in]  msg   The message
 */
void ArtiHardware::diffCmdCallback(const arti_msgs::DiffCmd::ConstPtr& msg)
{
	ROS_INFO_ONCE("Arti Hardware Get Diff Command");
	// diffToLR(msg->linear.x, msg->angular.z, cmd_left_, cmd_right_);
	cmd_left_ = msg->left;
	cmd_right_ = msg->right;
	thresholdVelocity();
	cmd_time_ = ros::Time::now();
}


/**
 * @brief      integration function using Runge Kutta
 *
 * @param[in]  linear   The linear
 * @param[in]  angular  The angular
 */
void ArtiHardware::integrateRungeKutta2(const double& linear, const double& angular)
{
	const double direction = theta_ + angular * 0.5;

	/// Runge-Kutta 2nd order integration:
	px_       += linear * cos(direction);
	py_       += linear * sin(direction);
	theta_ += angular;
}

/**
 * @brief      Other possible integration method provided by the class
 *
 * @param      linear   The linear
 * @param      angular  The angular
 */
void ArtiHardware::integrateExact(const double& linear, const double& angular)
{
	if (fabs(angular) < 1e-6)
		integrateRungeKutta2(linear, angular);
	else
	{
		/// Exact integration (should solve problems when angular is zero):
		const double theta_old = theta_;
		const double r = linear / angular;
		theta_ += angular;
		px_       +=  r * (sin(theta_) - sin(theta_old));
		py_       += -r * (cos(theta_) - cos(theta_old));
	}
}


/**
 * @brief      threshold veloctiy
 */
void ArtiHardware::thresholdVelocity()
{
	if (cmd_left_ > maximum_vel_) {
		cmd_left_ = maximum_vel_;
	}

	if (cmd_left_ < -maximum_vel_) {
		cmd_left_ = -maximum_vel_;
	}

	if (cmd_right_ > maximum_vel_) {
		cmd_right_ = maximum_vel_;
	}

	if (cmd_right_ < -maximum_vel_) {
		cmd_right_ = -maximum_vel_;
	}
}

/**
 * @brief      diffrential to Left and Right value
 *
 * @param[in]  vx    linear velocity
 * @param[in]  wz    angular velocity
 * @param      vl    left velocity
 * @param      vr    right velocity
 */
void ArtiHardware::diffToLR(const double& vx, const double& wz, double& vl, double& vr)
{
	vl = vx - body_width_ / 2 * wz;
	vr = vx + body_width_ / 2 * wz;
}

/**
 * @brief      Left and Right value to differential value
 *
 * @param[in]  vl    left velocity
 * @param[in]  vr    right velocity
 * @param      vx    linear velocity
 * @param      wz    angular velocity
 */
void ArtiHardware::LRtoDiff(const double& vl, const double& vr, double& vx, double& wz)
{
	vx  = (vr + vl) * 0.5 ;
	wz = (vr - vl) / body_width_;
}

/**
 * @brief      Sets the initial pose.
 *
 * @param[in]  x      x postion
 * @param[in]  y      y position
 * @param[in]  theta  The theta
 */
void ArtiHardware::setPose(const double&x, const double& y, const double& theta)
{
	px_ = x;
	py_ = y;
	theta_ = theta;
}

}  // namespace arti_hadware

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "arti_base");
	ros::NodeHandle nh, private_nh("~");
	arti_hardware::ArtiHardware arti(nh, private_nh);

	// ros::spin();

	return 0;
}
