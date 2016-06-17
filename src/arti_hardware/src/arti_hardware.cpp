#include <arti_hardware/arti_hardware.h>
typedef boost::chrono::steady_clock time_source;

namespace arti_hardware
{

ArtiHardware::ArtiHardware(ros::NodeHandle nh, ros::NodeHandle private_nh): nh_(nh)
{
	cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ArtiHardware::cmdVelCallback, this, _1));
	diff_cmd_sub_ = nh.subscribe<arti_msgs::DiffCmd>("diff_cmd_vel", 1, boost::bind(&ArtiHardware::diffCmdCallback, this, _1));

	private_nh.param("port", port_, std::string("/dev/ttyACM0"));
	private_nh.param("body_width", body_width_, 1.0);
	private_nh.param("baud_rate", baud_rate_, 9600);
	private_nh.param("serial_time_out", serial_time_out_, 100);
	private_nh.param("control_rate", control_rate_, 30.0);
	private_nh.param("odom_rate", odom_rate_, 50.0);
	private_nh.param("odom_window", odom_window_, 5);
	private_nh.param("cmd_time_out", cmd_time_out_, 0.5);
	private_nh.param("wheel_multiplier", wheel_multiplier_, 0.5);
	private_nh.param("maximum_vel", maximum_vel_, 1.0);
	private_nh.param("odom_bias", odom_bias_, 1.0);
	private_nh.param("maximum_vel", maximum_vel_, 1.0);
	private_nh.param("flip_lr", flip_lr_, false);

	ROS_INFO("Arti Hardware got port %s", port_.c_str());
	ROS_INFO("Set Serial Timeout %d ms", serial_time_out_);
	ROS_INFO("Baud Rate %d", baud_rate_);
	ROS_INFO("Control Rate %f", control_rate_);
	ROS_INFO("Command Time out is %f s", cmd_time_out_);

	if (cmd_from_hardware_) {
		cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ArtiHardware::cmdVelCallback, this, _1));
	}

	serial::Timeout to = serial::Timeout::simpleTimeout(serial_time_out_);
	// serial::Timeout to(serial::Timeout::max(), serial_time_out_, serial_time_out_, serial_time_out_, serial_time_out_);
	serial_ = new serial::Serial(port_, baud_rate_, to, serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none);
	diff_odom_pub_ = nh.advertise<arti_msgs::DiffOdom>("/diff_odom", 1);
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);

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
	registerControlInterfaces();
	odom_thread_ = new boost::thread(boost::bind(&ArtiHardware::odomLoop, this));
	// control_thread_ = new boost::thread(boost::bind(&ArtiHardware::controlLoop, this));
}

ArtiHardware::~ArtiHardware()
{

	if (odom_thread_ != NULL) {
		odom_thread_->interrupt();
		odom_thread_->join();
		delete odom_thread_;
	}
	if (control_thread_ != NULL) {
		control_thread_->interrupt();
		control_thread_->join();
		delete control_thread_;
	}

	try
	{
		sendMotorCmd(0, 0);
		serial_->close();
		ROS_INFO("Serial port shutting down");
	}
	catch (std::exception ex) {};
	delete serial_;
	delete odom_thread_;
	delete control_thread_;
}

void ArtiHardware::test()
{
	ros::Rate r(control_rate_);
	while (nh_.ok()) {
		std::string tmpStr;
		if (serial_->isOpen()) {
			try
			{
				tmpStr = serial_->readline(100);
				std::cout << tmpStr << std::endl;
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
		} else {
			ROS_WARN("Serial is not open\n");
		}

	}
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
		// double inter_time = (ros::Time::now() - cmd_time_).toSec();
		// if ( inter_time > cmd_time_out_ ) {
		// 	cmd_left_ = 0;
		// 	cmd_right_ = 0;
		// 	// std::cout << "inter time: " << inter_time << std::endl;
		// 	cmd_time_ = ros::Time::now();
		// }
		// sendMotorCmd(cmd_left_, cmd_right_);
		std::cout << "Left Joints: " << joints_[0].velocity_command << " Right Joints: " << joints_[1].velocity_command << std::endl;
		sendMotorCmd(joints_[0].velocity_command, joints_[1].velocity_command);
		r.sleep();
	}
}

void ArtiHardware::commandHardware()
{
	// std::cout << "Left Joints: " << joints_[0].velocity_command << " Right Joints: " << joints_[1].velocity_command << std::endl;
	sendMotorCmd(joints_[0].velocity_command, joints_[1].velocity_command);
}

void ArtiHardware::odomLoop()
{
	ROS_INFO_ONCE("Start to publish odom");
	ros::Rate r(odom_rate_);
	std::string dataStr;
	std::string tmpStr;
	int num = 0;
	int left = 0, right = 0;
	unsigned char token[1];
	while (nh_.ok()) {

		if (serial_->available() && serial_->isOpen()) {
			try
			{
				tmpStr = serial_->readline(20, "ODOMS,");
				dataStr = serial_->readline(20, "ODOME\n");
				parseOdomStr(dataStr, left, right);
			}
			catch (serial::SerialException ex) //No data received
			{
				ROS_WARN("Serial read exception: %s", ex.what());
			}
			catch (std::runtime_error ex)
			{
				ROS_WARN("Serial read exception: %s", ex.what());
			}
		}
		processOdom(left, right);
		// printOdom(odom_queue_.back());
		dataStr = "";
		tmpStr = "";
		num = 0;
		r.sleep();
	}
}

void ArtiHardware::printOdom(const DiffOdom& odom)
{
	std::cout << "left travel: " << odom.left_travel << " right travel: " <<
	          odom.right_travel << " left speed: " << odom.left_speed << " right speed:" << odom.right_speed
	          << std::endl;
}

void ArtiHardware::processOdom(const int& left, const int& right)
{
	DiffOdom odom;
	odom.left_travel = left * wheel_multiplier_;
	odom.right_travel = right * wheel_multiplier_;
	odom.header.stamp = ros::Time::now();
	// if there is not enough data in the stack add it to the queue
	if (odom_queue_.size() == odom_window_) {
		double time_inter = (odom.header.stamp - odom_queue_.front().header.stamp).toSec();
		// std::cout << time_inter << std::endl;
		odom.left_speed = (odom.left_travel - odom_queue_.front().left_travel) / time_inter;
		odom.right_speed = (odom.right_travel - odom_queue_.front().right_travel) / time_inter;
		odom_queue_.pop();
	}
	odom_queue_.push(odom);
	diff_odom_pub_.publish(odom);
	joints_[0].position = odom.left_travel;
	joints_[1].position = odom.right_travel;
	// joints_[0].velocity = odom.left_speed;
	// joints_[1].velocity = odom.right_speed;
}

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

void ArtiHardware::registerControlInterfaces()
{
	std::vector<std::string> joint_names;
	joint_names.push_back(std::string("front_left_wheel"));
	joint_names.push_back(std::string("front_right_wheel"));

	for (unsigned int i = 0; i < joint_names.size(); i++)
	{
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
		        &joints_[i].position, &joints_[i].velocity,
		        &joints_[i].effort);
		joint_state_interface_.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(
		    joint_state_handle, &joints_[i].velocity_command);
		velocity_joint_interface_.registerHandle(joint_handle);
	}
	registerInterface(&joint_state_interface_);
	registerInterface(&velocity_joint_interface_);
}

// void ArtiHardware::integrateExact(double linear, double angular)
// {
// 	if (fabs(angular) < 1e-6)
// 		integrateRungeKutta2(linear, angular);
// 	else
// 	{
// 		/// Exact integration (should solve problems when angular is zero):
// 		const double heading_old = heading_;
// 		const double r = linear / angular;
// 		heading_ += angular;
// 		x_       +=  r * (sin(heading_) - sin(heading_old));
// 		y_       += -r * (cos(heading_) - cos(heading_old));
// 	}
// }

// void Odometry::integrateRungeKutta2(double linear, double angular)
// {
// 	const double direction = heading_ + angular * 0.5;

// 	x_       += linear * cos(direction);
// 	y_       += linear * sin(direction);
// 	heading_ += angular;
// }

/**
 * @brief      { Send the motor command }
 *
 * @param[in]  left   The left command
 * @param[in]  right  The right command
 */
void ArtiHardware::sendMotorCmd(const double& left, const double right)
{
	if (!serial_->isOpen())
	{
		ROS_ERROR("Serial port not available");
		return;
	}

	sprintf(cmd_, "\nMOTOS,%d,%d,MOTOE\n", (int) left, (int) right);
	try
	{

		serial_->write(cmd_); //Send query
		// if (left != 0.0 && right != 0.0) {
		// 	std::cout << cmd_;
		// }
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
 * @brief      { function_description }
 *
 * @param[in]  msg   The message
 */
void ArtiHardware::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO_ONCE("Arti Hardware Get Command");
	// diffToLR(msg->linear.x, msg->angular.z, cmd_left_, cmd_right_);
	diffToLR(msg->linear.x, msg->angular.z, joints_[0].velocity_command, joints_[1].velocity_command);
	cmd_time_ = ros::Time::now();
}

void ArtiHardware::diffToLR(const double& vx, const double& wz, double& left, double& right)
{
	left = ( vx - body_width_ / 2 * wz ) * 127;
	right = ( vx + body_width_ / 2 * wz ) * 127;
}

}  // namespace arti_hadware

// #include "controller_manager/controller_manager.h"
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "arti_base");
	ros::NodeHandle nh, private_nh("~");
	arti_hardware::ArtiHardware arti(nh, private_nh);
	controller_manager::ControllerManager cm(&arti);
	ros::Rate r(50);
	time_source::time_point last_time = time_source::now();
	time_source::time_point this_time = time_source::now();
	while (nh.ok()) {
		// ROS_INFO("aaaa");
		this_time = time_source::now();
		boost::chrono::duration<double> elapsed_duration = this_time - last_time;
		ros::Duration elapsed(elapsed_duration.count());
		last_time = this_time;
		cm.update(ros::Time::now(), elapsed);
		arti.commandHardware();
		r.sleep();
		ros::spinOnce();
	}
	ros::spin();

	return 0;
}
