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
#include <nav_msgs/Odometry.h>
// #include <SerialStream.h>

namespace arti_hardware
{

class ArtiHardware {

private:
	serial::Serial* serial_;
	// boost::asio::serial_port* serial_;
	std::string port_;
	double body_width_;
	ros::Subscriber cmd_sub_, diff_cmd_sub_;
	char cmd_[10];
	int baud_rate_;
	int serial_time_out_;
	ros::NodeHandle nh_;
	double control_rate_;
	double odom_rate_;
	double cmd_left_;
	double cmd_right_;
	ros::Time cmd_time_;
	double cmd_time_out_;
	boost::thread* odom_thread_;
	boost::mutex serial_mutex_;
	std::queue<arti_msgs::DiffOdom> diff_odom_queue_;
	int odom_window_;
	ros::Publisher diff_odom_pub_, odom_pub_;
	double wheel_multiplier_;
	double maximum_vel_;
	double odom_bias_;
	bool flip_lr_;
	double px_, py_, theta_;
	double vx_, wz_;
	double vl_, vr_;
	arti_msgs::DiffOdom diff_odom_old_;

public:
	ArtiHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);
	~ArtiHardware();
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void controlLoop();
	void odomLoop();
	void sendMotorCmd(const double& left, const double right);
	void test();
	bool parseOdomStr(const std::string& str, int& left, int& right);
	void printOdom(const arti_msgs::DiffOdom& odom);
	void processOdom(const int& left, const int& right);
	void diffCmdCallback(const arti_msgs::DiffCmd::ConstPtr& msg);
	void thresholdVelocity();
	void integrateRungeKutta2(const double& linear, const double& angular);
	void integrateExact(const double& linear, const double& angular);
	void diffToLR(const double& vx, const double& wz, double& vl, double& vr);
	void LRtoDiff(const double& vl, const double& vr, double& vx, double& wz);
	void setPose(const double&x, const double& y, const double& theta);
};

}