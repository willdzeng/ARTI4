#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <string>
#include <queue>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <arti_msgs/DiffOdom.h>
#include <arti_msgs/DiffCmd.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/chrono.hpp>
#include <nav_msgs/Odometry.h>
// #include <SerialStream.h>

namespace arti_hardware
{

class ArtiHardware : public hardware_interface::RobotHW {

private:
	serial::Serial* serial_;
	// boost::asio::serial_port* serial_;

	ros::Subscriber cmd_sub_, diff_cmd_sub_;
	ros::Publisher diff_odom_pub_, odom_pub_;
	ros::Time cmd_time_;
	ros::NodeHandle nh_;

	hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::VelocityJointInterface velocity_joint_interface_;

	char cmd_[10];
	int baud_rate_;
	int serial_time_out_;
	int odom_window_;
	std::string port_;
	std::queue<arti_msgs::DiffOdom> odom_queue_;
	double body_width_;
	double control_rate_;
	double odom_rate_;
	double cmd_left_;
	double cmd_right_;
	double cmd_time_out_;
	double wheel_multiplier_;
	double maximum_vel_;
	double odom_bias_;

	boost::thread* odom_thread_;
	boost::thread* control_thread_;
	boost::mutex serial_mutex_;

	
	bool cmd_from_hardware_;
	bool flip_lr_;

	struct Joint
	{
		double position;
		double position_offset;
		double velocity;
		double effort;
		double velocity_command;

		Joint() :
			position(0), velocity(0), effort(0), velocity_command(0)
		{}
	}

	joints_[2];

public:
	ArtiHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);
	~ArtiHardware();
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void diffCmdCallback(const arti_msgs::DiffCmd::ConstPtr& msg);
	void diffToLR(const double& vx, const double& wz, double& left, double& right);
	void controlLoop();
	void odomLoop();
	void sendMotorCmd(const double& left, const double right);
	void test();
	bool parseOdomStr(const std::string& str, int& left, int& right);
	void printOdom(const arti_msgs::DiffOdom& odom);
	void processOdom(const int& left, const int& right);
	void registerControlInterfaces();
	void commandHardware();
	void thresholdVelocity();
};

}