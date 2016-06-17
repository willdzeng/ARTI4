#include <arti_control/arti_control.h>

namespace arti_control
{

ArtiControl::ArtiControl(ros::NodeHandle nh, ros::NodeHandle private_nh): nh_(nh)
{
	diff_odom_sub_ = nh.subscribe<arti_msgs::DiffOdom>("diff_odom", 1, boost::bind(&ArtiControl::diffOdomCallback, this, _1));
	cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	diff_cmd_pub_ = nh.advertise<arti_msgs::DiffCmd>("diff_cmd_vel", 1);

	private_nh.param("running_time", running_time_, 5.0);
	private_nh.param("body_width", body_width_, 5.0);
	private_nh.param("target_forward_vel", target_forward_vel_, 1.0);
	private_nh.param("target_turning_vel", target_turning_vel_, 0.0);
	private_nh.param("cmd_rate", cmd_rate_, 50.0);
	private_nh.param("kp", kp_, 1.0);
	private_nh.param("ki", ki_, 1.0);
	private_nh.param("kd", kd_, 1.0);
	private_nh.param("kd_window", kd_window_, 5);
	private_nh.param("maximum_vel", maximum_vel_, 1.0);
	private_nh.param("set_initial_odom", set_initial_odom_, true);
	diffToLR(target_forward_vel_, target_turning_vel_, left_cmd_, right_cmd_);
	// left_cmd_ = target_forward_vel_;
	// right_cmd_ = target_forward_vel_;
	controlLogic();
}

ArtiControl::~ArtiControl()
{

}

/**
 * @brief      the main control loop of a the hardware
 */
void ArtiControl::controlLogic()
{
	goStraight(running_time_);
}

void ArtiControl::goStraight(double time)
{
	ros::Rate r(cmd_rate_);
	ros::Time start_time = ros::Time::now();
	ros::Time end_time = start_time + ros::Duration(time);
	odom_diff_ = 0;
	if (set_initial_odom_) {
		// wait until get the first odom
		while (odom_queue_.size() == 0) {
			r.sleep();
			ros::spinOnce();
		}
		// record the initial differences
		odom_diff_ = odom_queue_.back().left_travel - odom_queue_.back().right_travel;
	}


	while (nh_.ok() && ros::Time::now() < end_time) {
		ros::spinOnce();
		straightLineControl();
		arti_msgs::DiffCmd cmd;
		cmd.left = left_cmd_;
		cmd.right = right_cmd_;
		diff_cmd_pub_.publish(cmd);
		r.sleep();
	}
}

void ArtiControl::straightLineControl()
{
	if (odom_queue_.size() == kd_window_) {
		arti_msgs::DiffOdom* back_odom, *front_odom;
		back_odom = &odom_queue_.back();
		front_odom = &odom_queue_.front();
		double dt = (back_odom->header.stamp - front_odom->header.stamp).toSec();
		double dp = (back_odom->left_travel - back_odom->right_travel - odom_diff_);
		double dd = ( (back_odom->left_travel - back_odom->right_travel)
		              - (front_odom->left_travel - front_odom->right_travel)) / dt;
		double vp = kp_ * dp;
		double vd = kd_ * dd;
		// left_cmd_ -= (vp + vd);
		// left_cmd_ += ki_ * (target_forward_vel_ - left_cmd_);
		// right_cmd_ += (vp + vd);
		// right_cmd_ += ki_ * (target_forward_vel_ - right_cmd_);
		left_cmd_ = target_forward_vel_ - vp - vd;
		right_cmd_ = target_forward_vel_ + vp + vd;

		if (left_cmd_ > maximum_vel_) {
			left_cmd_ = maximum_vel_;
		}

		if (left_cmd_ < -maximum_vel_) {
			left_cmd_ = -maximum_vel_;
		}

		if (right_cmd_ > maximum_vel_) {
			right_cmd_ = maximum_vel_;
		}

		if (right_cmd_ < -maximum_vel_) {
			right_cmd_ = -maximum_vel_;
		}

		std::cout << "kp:" << vp << " kd:" << vd << " dp:" << dp << " dd:" << dd << " left: " << left_cmd_ << " right: " << right_cmd_ << std::endl;
	}

}
/**
 * @brief      { function_description }
 *
 * @param[in]  msg   The message
 */
void ArtiControl::diffOdomCallback(const arti_msgs::DiffOdom::ConstPtr& msg)
{
	ROS_INFO_ONCE("Arti Control Get Odom");
	arti_msgs::DiffOdom odom = *msg;
	odom_queue_.push(odom);
	if (odom_queue_.size() > kd_window_) {
		odom_queue_.pop();
	}
}

void ArtiControl::diffToLR(const double& vx, const double& wz, double& left, double& right)
{
	left = vx - body_width_ / 2 * wz ;
	right = vx + body_width_ / 2 * wz ;
}

void ArtiControl::LRToDiff(const double& left, const double& right, double& vx, double& wz)
{
	vx = (right + left) / 2;
	wz = (right - left) / body_width_;
}


}  // namespace arti_control

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "arti_control");
	ros::NodeHandle nh, private_nh("~");
	arti_control::ArtiControl arti(nh, private_nh);

	// ros::spin();

	return 0;
}
