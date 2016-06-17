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
// #include <SerialStream.h>

namespace arti_control
{

class ArtiControl {

private:
	double running_time_;
	arti_msgs::DiffOdom odom_;
	ros::Subscriber diff_odom_sub_;
	ros::Publisher cmd_pub_, diff_cmd_pub_;
	double forward_vel_;
	double turning_vel_;
	double target_forward_vel_;
	double target_turning_vel_;
	double body_width_;
	ros::NodeHandle nh_;
	double cmd_rate_;
	double kp_,ki_,kd_;
	int kd_window_;
	std::queue<arti_msgs::DiffOdom> odom_queue_;
	double left_cmd_;
	double right_cmd_;
	double maximum_vel_;
	double odom_diff_;
	bool set_initial_odom_;

public:
	ArtiControl(ros::NodeHandle nh, ros::NodeHandle private_nh);
	~ArtiControl();
	void controlLogic();
	void straightLineControl();

	void diffOdomCallback(const arti_msgs::DiffOdom::ConstPtr& msg);

	void diffToLR(const double& vx, const double& wz, double& left, double& right);
	void goStraight(double time);
	void LRToDiff(const double& left, const double& right, double& vx, double& wz);

};

}