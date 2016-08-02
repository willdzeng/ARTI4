
#ifndef MACHINE_LEARNING_SENSOR_DATA_COLLECTOR_
#define MACHINE_LEARNING_SENSOR_DATA_COLLECTOR_

#include <ros/ros.h>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>


#include <vector>
#include <string>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
// #include <base_local_planner/odometry_helper_ros.h>
// #include <base_local_planner/trajectory.h>

// #include <trajectory_msgs/Trajectory.h>
// #include <dynamic_reconfigure/server.h>
#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <nav_msgs/Odometry.h>
// #include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <arti_msgs/Temperature.h>
#include <arti_msgs/Ultrasound.h>
// #include <ugv_msgs/PowerStatus.h>
// #include <ugv_msgs/DifferentialCmd.h>
// #include <ugv_msgs/DifferentialOdom.h>
// #include <diagnostic_msgs/DiagnosticStatus.h>
// #include <diagnostic_msgs/KeyValue.h>
//#include <std_msgs/Bool.h>

#include <fstream>

// #include <machine_learning/MP_generator_config.h>
// #include <motion_primitive_generator/vel_sample_generator.h>


namespace machine_learning {

class SensorDataRecorder {

public:
	SensorDataRecorder(tf::TransformListener& tf, ros::NodeHandle& nh);

	~SensorDataRecorder();

	bool recordData();

	void getRobotPoseTFthread();

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

	void EKFCallback(const nav_msgs::Odometry::ConstPtr& msg);

	void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

	void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg);

	void cmdCallBack(const geometry_msgs::Twist::ConstPtr& msg);

	void tempCallBack(const arti_msgs::Temperature::ConstPtr& msg);

	void ultraCallBack(const arti_msgs::Ultrasound::ConstPtr& msg);

	// void powerStatusCallback(const ugv_msgs::PowerStatus::ConstPtr& msg);

	// void cmdDiffCallback(const ugv_msgs::DifferentialCmd::ConstPtr& msg);

	// void diffOdomCallback(const ugv_msgs::DifferentialOdom::ConstPtr& msg);

private:
	ros::NodeHandle& nh_;
	tf::TransformListener& tf_;
	boost::thread* generator_thread_;
	boost::thread* tf_thread_;

	ros::Subscriber odom_sub_, state_sub_, imu_sub_, diagnostic_sub_, gps_sub_, ekf_sub_, cmd_sub_;
	ros::Subscriber power_sub_, diff_cmd_sub_, diff_odom_sub_;
	ros::Subscriber temp_sub_, ultra_sub_;

	double Vx_odom_, Vy_odom_, Vz_odom_;
	double omega_x_odom_, omega_y_odom_, omega_z_odom_;
	double Px_odom_, Py_odom_, Pz_odom_;
	double theta_x_odom_, theta_y_odom_, theta_z_odom_;

	double cmd_vx_, cmd_wz_;

	double Px_tf_, Py_tf_, Pz_tf_;
	double theta_x_tf_, theta_y_tf_, theta_z_tf_;

	double Px_ekf_, Py_ekf_, Pz_ekf_;
	double theta_x_ekf_, theta_y_ekf_, theta_z_ekf_;
	double Vx_ekf_, Vy_ekf_, Vz_ekf_;
	double omega_x_ekf_, omega_y_ekf_, omega_z_ekf_;

	double acc_x_, acc_y_, acc_z_;
	double alpha_x_, alpha_y_, alpha_z_;
	double Ul_, Ur_, Il_, Ir_;
	double latitude_, longtitude_, altitude_;

	double record_frequency_;

	double cmd_diff_left_, cmd_diff_right_;
	double left_travel_, right_travel_, left_speed_, rigth_speed_;

	std::string odom_topic_, ekf_topic_, cmd_vel_topic_, states_topic_, imu_topic_, diagnostic_topic_, gps_topic_;
	std::string robot_namespace_, status_name_, body_link_name_, odom_link_name_;
	std::string output_file_;

	std::string power_topic_, cmd_diff_topic_, diff_odom_topic_;

	bool record_tf_;
	std::vector<float> temps_;
	std::vector<float> ultra_dists_;

};


}

#endif
