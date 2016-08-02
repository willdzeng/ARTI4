#include <sensor_data_recorder/sensor_data_recorder.h>

#define PI 3.141592654

namespace machine_learning {


SensorDataRecorder::SensorDataRecorder(tf::TransformListener& tf, ros::NodeHandle& nh) :
	tf_(tf),
	nh_(nh)
{

	ros::NodeHandle private_nh("sensor_data_recorder");
	private_nh.param("odom_topic", odom_topic_, std::string("/odom"));
	private_nh.param("ekf_topic", ekf_topic_, std::string("/odometry/filtered"));
	private_nh.param("imu_topic_", imu_topic_, std::string("/imu/data"));
	private_nh.param("diagnostic_topic", diagnostic_topic_, std::string("/diagnostics"));
	private_nh.param("robot_namespace", robot_namespace_, std::string(""));
	private_nh.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
	private_nh.param("body_link_name", body_link_name_, std::string("base_link"));
	private_nh.param("odom_link_name", odom_link_name_, std::string("odom"));
	private_nh.param("status_name", status_name_, std::string("husky_node: system_status"));
	private_nh.param("gps_topic", gps_topic_, std::string("/gps"));
	private_nh.param("output_file", output_file_, std::string("/home/umdugv/sensor_data.csv"));
	private_nh.param("power_topic", power_topic_, std::string("/husky/power_status"));
	private_nh.param("cmd_diff_topic", cmd_diff_topic_, std::string("/husky/cmd_diff"));
	private_nh.param("diff_odom_topic", diff_odom_topic_, std::string("/husky/diff_odom"));
	private_nh.param("record_frequency", record_frequency_, 30.0);
	private_nh.param("record_tf", record_tf_, false);

	ROS_INFO("SensorDataRecorder got robot_namespace: %s", robot_namespace_.c_str());
	ROS_INFO("SensorDataRecorder got output file name: %s", output_file_.c_str());

	// odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(robot_namespace_ + odom_topic_, 1,  boost::bind(&SensorDataRecorder::odomCallback, this, _1));
	// ekf_sub_ = nh_.subscribe<nav_msgs::Odometry>(robot_namespace_ + ekf_topic_, 1,  boost::bind(&SensorDataRecorder::EKFCallback, this, _1));
	// imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(robot_namespace_ + imu_topic_, 1,  boost::bind(&SensorDataRecorder::imuCallback, this, _1));
	// diagnostic_sub_ = nh_.subscribe<diagnostic_msgs::DiagnosticArray>(robot_namespace_ + diagnostic_topic_, 1,  boost::bind(&SensorDataRecorder::diagnosticCallback, this, _1));

	// power_sub_ = nh_.subscribe<ugv_msgs::PowerStatus>(robot_namespace_ + power_topic_, 1,  boost::bind(&SensorDataRecorder::powerStatusCallback, this, _1));
	// diff_cmd_sub_ = nh_.subscribe<ugv_msgs::DifferentialCmd>(robot_namespace_ + cmd_diff_topic_, 1,  boost::bind(&SensorDataRecorder::cmdDiffCallback, this, _1));
	// diff_odom_sub_ = nh_.subscribe<ugv_msgs::DifferentialOdom>(robot_namespace_ + diff_odom_topic_, 1,  boost::bind(&SensorDataRecorder::diffOdomCallback, this, _1));

	// gps_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(robot_namespace_ + gps_topic_, 1,  boost::bind(&SensorDataRecorder::gpsCallBack, this, _1));
	cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1,  boost::bind(&SensorDataRecorder::cmdCallBack, this, _1));
	temp_sub_ = nh_.subscribe<arti_msgs::Temperature>("temperature", 1,  boost::bind(&SensorDataRecorder::tempCallBack, this, _1));
	ultra_sub_ = nh_.subscribe<arti_msgs::Ultrasound>("ultrasound", 1,  boost::bind(&SensorDataRecorder::ultraCallBack, this, _1));


	if (record_tf_) {
		tf_thread_ = new boost::thread(boost::bind(&SensorDataRecorder::getRobotPoseTFthread, this));
	}

	generator_thread_ = new boost::thread(boost::bind(&SensorDataRecorder::recordData, this));
}


SensorDataRecorder::~SensorDataRecorder()
{
	if (tf_thread_ != NULL) {
		delete tf_thread_;
	}
	if (generator_thread_ != NULL) {
		delete generator_thread_;
	}
}


bool SensorDataRecorder::recordData()
{
	ros::NodeHandle n;
	std::ofstream file(output_file_.c_str(), std::ofstream::out);
	ros::Rate r(record_frequency_);

	ros::Time start_time = ros::Time::now();
	ros::Duration inter_time;
	double time_num;

	while (n.ok()) {
		ROS_INFO_ONCE("Start Recording");
		ros::spinOnce();
		inter_time = ros::Time::now() - start_time;
		time_num = inter_time.toSec();

		file << time_num << " ";

		file << cmd_vx_ << " " << cmd_wz_ << " ";

		for (int i = 0; i < temps_.size(); i++) {
			file << temps_[i] << " ";
		}

		for (int i = 0; i < ultra_dists_.size(); i++) {
			file << ultra_dists_[i] << " ";
		}

		// file << cmd_diff_left_ << " " << cmd_diff_right_ << " ";

		// file << Px_tf_ << " " << Py_tf_ << " " << Pz_tf_ << " ";

		// file << theta_x_tf_ << " " << theta_y_tf_ << " " << theta_z_tf_ <<" ";

		// file << Px_ekf_ << " " << Py_ekf_ << " " << Pz_ekf_ << " ";

		// file << theta_x_ekf_ << " " << theta_y_ekf_ << " " << theta_z_ekf_ << " ";

		// file << Vx_ekf_ << " " << Vy_ekf_ << " " <<  Vz_ekf_ << " ";

		// file << omega_x_ekf_ << " " << omega_y_ekf_ << " " <<  omega_z_ekf_ << " ";

		// file << Px_odom_ << " " << Py_odom_ << " " << theta_z_odom_ << " ";

		// file << Vx_odom_ << " " << omega_z_odom_ << " ";

		// file << left_travel_ << " " << right_travel_ << " " << left_speed_ << " " << rigth_speed_ << " ";

		// file << acc_x_ << " " << acc_y_ << " " << acc_z_ << " ";

		// file << latitude_ << " " << longtitude_ << " " << altitude_ << " ";

		// file << Ul_ << " " << Ur_ << " " ;

		// file << Il_ << " " << Ir_ << " " ;

		file << std::endl;

		r.sleep();
	}
	file.close();
	ROS_INFO("Record Finished");
}

void SensorDataRecorder::getRobotPoseTFthread()
{
	ros::NodeHandle n;
	ros::Rate r(record_frequency_);
	ROS_INFO_ONCE("Setup TF thread, reading tf pose");
	while (n.ok()) {
		ros::Time tTime;
		std::string error;
		tf::StampedTransform transform;
		try
		{
			ros::Time now = ros::Time::now();
			tf_.waitForTransform(robot_namespace_ + odom_link_name_,
			                     robot_namespace_ + body_link_name_,
			                     ros::Time(),
			                     ros::Duration(1.0));
			tf_.lookupTransform(robot_namespace_ + odom_link_name_,
			                    robot_namespace_ + body_link_name_,
			                    ros::Time(),
			                    transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_WARN("tf failure: %s", ex.what());
			continue;
		}
		// get position
		Px_tf_ = transform.getOrigin().getX();
		Py_tf_ = transform.getOrigin().getY();
		Pz_tf_ = transform.getOrigin().getZ();
		// get angle
		tf::Matrix3x3 m(transform.getRotation());
		m.getRPY(theta_x_tf_, theta_y_tf_, theta_z_tf_);

	}

}

void SensorDataRecorder::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO_ONCE("odom received!");
	Px_odom_ = msg->pose.pose.position.x;
	Py_odom_ = msg->pose.pose.position.y;
	Pz_odom_ = msg->pose.pose.position.z;
	theta_x_odom_ = msg->pose.pose.orientation.x;
	theta_y_odom_ = msg->pose.pose.orientation.y;
	theta_z_odom_ = msg->pose.pose.orientation.z;
	Vx_odom_ = msg->twist.twist.linear.x;
	Vy_odom_ = msg->twist.twist.linear.y;
	Vz_odom_ = msg->twist.twist.linear.z;
	omega_x_odom_ = msg->twist.twist.angular.x;
	omega_y_odom_ = msg->twist.twist.angular.y;
	omega_z_odom_ = msg->twist.twist.angular.z;
}

void SensorDataRecorder::EKFCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO_ONCE("EKF odom received!");
	Vx_ekf_ = msg->twist.twist.linear.x;
	Vy_ekf_ = msg->twist.twist.linear.y;
	Vz_ekf_ = msg->twist.twist.linear.z;
	omega_x_ekf_ = msg->twist.twist.angular.x;
	omega_y_ekf_ = msg->twist.twist.angular.y;
	omega_z_ekf_ = msg->twist.twist.angular.z;
	Px_ekf_ = msg->pose.pose.position.x;
	Py_ekf_ = msg->pose.pose.position.y;
	Pz_ekf_ = msg->pose.pose.position.z;
	theta_x_ekf_ = msg->pose.pose.orientation.x;
	theta_y_ekf_ = msg->pose.pose.orientation.y;
	theta_z_ekf_ = msg->pose.pose.orientation.z;
}


void SensorDataRecorder::diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
	ROS_INFO_ONCE("diagnostic received!");
	bool found_name = false;
	for (int i = 0; i < msg->status.size(); i++) {
		if (status_name_.compare(msg->status[i].name) == 0 ) {
			found_name = true;
			Ul_ = std::stod(msg->status[i].values[2].value);
			Ur_ = std::stod(msg->status[i].values[3].value);
			Il_ = std::stod(msg->status[i].values[5].value);
			Ir_ = std::stod(msg->status[i].values[6].value);
			ROS_INFO_ONCE("found the desired status info");
		}
	}
}

void SensorDataRecorder::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	ROS_INFO_ONCE("imu received!");
	acc_x_ = msg->linear_acceleration.x;
	acc_y_ = msg->linear_acceleration.y;
	acc_z_ = msg->linear_acceleration.z;
	// alpha_x_ = msg->angular_acceleration.x;
	// alpha_y_ = msg->angular_acceleration.y;
	// alpha_z_ = msg->angular_acceleration.z;
}


void SensorDataRecorder::gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	ROS_INFO_ONCE("GPS received");
	latitude_ = msg->latitude;
	longtitude_ = msg->longitude;
	altitude_ = msg->altitude;
}

void SensorDataRecorder::cmdCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO_ONCE("Comand Velocity Received");
	cmd_vx_ = msg->linear.x;
	cmd_wz_ = msg->angular.z;
}

void SensorDataRecorder::tempCallBack(const arti_msgs::Temperature::ConstPtr& msg)
{
	ROS_INFO_ONCE("Got Arti Temperature");
	temps_ = msg->value;
}

void SensorDataRecorder::ultraCallBack(const arti_msgs::Ultrasound::ConstPtr& msg)
{
	ROS_INFO_ONCE("Got Arti Ultrasound");
	ultra_dists_ = msg->distance;
}

// void SensorDataRecorder::powerStatusCallback(const ugv_msgs::PowerStatus::ConstPtr& msg)
// {
// 	ROS_INFO_ONCE("Power Status Received");
// 	Il_ = msg->left_current;
// 	Ir_ = msg->right_current;
// 	Ul_ = msg->left_voltage;
// 	Ur_ = msg->right_voltage;
// }

// void SensorDataRecorder::cmdDiffCallback(const ugv_msgs::DifferentialCmd::ConstPtr& msg)
// {
// 	ROS_INFO_ONCE("Diff Cmd Received");
// 	cmd_diff_left_ = msg->left_input;
// 	cmd_diff_right_ = msg->right_input;
// }

// void SensorDataRecorder::diffOdomCallback(const ugv_msgs::DifferentialOdom::ConstPtr& msg)
// {
// 	ROS_INFO_ONCE("Diff Odom Received");
// 	left_travel_ = msg->left_travel;
// 	right_travel_ = msg->right_travel;
// 	left_speed_ = msg->left_speed;
// 	rigth_speed_ = msg->right_speed;
// }


}; // end of namespace

