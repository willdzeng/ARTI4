#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace arti_navigation {
class StairClimb {
public:
	enum STATUS { READY, CONTROLLING, DONE, STAIRCLIMB };
	StairClimb(ros::NodeHandle nh, ros::NodeHandle private_nh) {
		nh_ = nh;
		private_nh_ = private_nh;
		maximum_point_pub_size_ = 20;
		goal_point_tolerance_ = 0.2;

		private_nh_.param("climb_time", climb_time_, 10.0);
		private_nh_.param("goal_index", goal_index_, 1);
		private_nh_.param("stair_climb_vel", stair_climb_vel_, 0.5);
		private_nh_.param("cmd_rate", cmd_rate_, 20.0);
		private_nh.param("distance_tolerance", dist_tolerance_, 0.5);
		private_nh.param("robot_namespace", robot_namespace_, std::string(""));
		private_nh.param("map_frame_name", map_frame_name_, std::string("map"));
		private_nh.param("body_link_name", body_link_name_, std::string("base_link"));

		point_sub_ = nh_.subscribe ("way_point", 10, &StairClimb::pointCallback, this);
		cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point", maximum_point_pub_size_);
		reset();
	}

	~StairClimb() {

	}

	void controlling() {
		int index = 0;
		while (nh_.ok() && index < goal_points_.size() && status_ == CONTROLLING) {
			if ( index == goal_index_ ) {
				shutdownMapping();
				goStraight(climb_time_, stair_climb_vel_);
				resumeMapping();
				goToGoal(index);
				index++;
			} else {
				goToGoal(index);
				index++;
			}
		}
	}

	void shutdownMapping(){

	}

	void resumeMapping(){
		
	}

	void pointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
		// ROS_INFO ( "Got point" );
		geometry_msgs::PoseStamped point;
		point = *msg;

		if ( status_ == READY ) {
			bool add = false;
			if ( !goal_points_.empty()) {
				// calcuate the euclidean distance
				double dist = distBetweenPose(point.pose, goal_points_.back().pose);
				if ( dist > goal_point_tolerance_ ) {
					add = true;
				}
			}
			else {
				add = true;
			}
			//only add the point if it is not too close. this is a hack to overcome lack of UI support in rviz
			if ( add == true ) {
				ROS_INFO("Add a new point %f %f %f", point.pose.position.x, point.pose.position.y, point.pose.orientation.z);
				goal_points_.push_back(point);
				// publish point on rviz;
				geometry_msgs::PointStamped ps;
				ps.point = point.pose.position;
				ps.header = point.header;
				point_pub_.publish(ps);
			}
			else {
				ROS_INFO("Finished points receiveing, goal size is %lu, start control", goal_points_.size());
				status_ = CONTROLLING;
				controlling();
				return;
			}
		}

		if (status_ == CONTROLLING) {
			ROS_INFO("Cancel control and Wait new points publishing");
			reset();
			ROS_INFO("Add a new point %f %f %f", point.pose.position.x, point.pose.position.y, point.pose.orientation.z);
			goal_points_.push_back(point);

		}
	}

	bool goToGoal(int goal_index) {
		ros::Rate r(cmd_rate_);
		geometry_msgs::PoseStamped current_goal = goal_points_[goal_index];
		goal_pub_.publish(current_goal);

		ROS_INFO("Go to goal index %d,Wait for the robot moving", goal_index);
		while (nh_.ok() && status_ == CONTROLLING) {
			updateRobotPose();
			double dist = distBetweenPose(current_goal.pose, current_pose_);
			ROS_INFO("Distance is %f", dist);
			if (dist < dist_tolerance_) {
				ROS_INFO("Reached the goal %d", goal_index);
				return true;
			}
			ros::spinOnce();
			r.sleep();
		}
	}

	bool goStraight(double time, double cmd_vel_x) {

		ros::Rate r(cmd_rate_);
		ros::Time start_time = ros::Time::now();
		ros::Time end_time = start_time + ros::Duration(time);
		ROS_INFO("Start Stair Climb");
		while (nh_.ok() && ros::Time::now() < end_time) {
			ros::spinOnce();
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = cmd_vel_x;
			cmd_pub_.publish(cmd_vel);
			r.sleep();
		}
		ROS_INFO("Stair Climb Finihsed");
		return true;
	}

	void reset() {
		goal_points_.clear();
		status_ = READY;
		for (int i = 0 ; i < maximum_point_pub_size_; i++) {
			geometry_msgs::PointStamped empty_point;
			point_pub_.publish(empty_point);
		}
	}

	double distBetweenPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
		return sqrt(pow((pose1.position.x - pose2.position.x), 2) +
		            pow((pose1.position.y - pose2.position.y), 2));
	}


	void updateRobotPose()
	{
		ros::Time tTime;
		std::string error;
		tf::StampedTransform transform;
		bool success = true;
		try
		{
			ros::Time now = ros::Time::now();
			tf_.waitForTransform(robot_namespace_ + map_frame_name_,
			                     robot_namespace_ + body_link_name_,
			                     ros::Time(),
			                     ros::Duration(1.0));
			tf_.lookupTransform(robot_namespace_ + map_frame_name_,
			                    robot_namespace_ + body_link_name_,
			                    ros::Time(),
			                    transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_WARN("tf failure: %s", ex.what());
			success = false;
		}
		if (success) {
			// geometry_msgs::PoseStamped pose;
			// tf::poseStampedTFToMsg(transform, pose);
			// current_pose_ = pose.pose;
			current_pose_.position.x = transform.getOrigin().getX();
			current_pose_.position.y = transform.getOrigin().getY();
			current_pose_.position.z = transform.getOrigin().getZ();
			current_pose_.orientation.x = transform.getRotation().getX();
			current_pose_.orientation.y = transform.getRotation().getY();
			current_pose_.orientation.z = transform.getRotation().getZ();
			current_pose_.orientation.w = transform.getRotation().getW();
		}
	}

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	double climb_time_, cmd_rate_, stair_climb_vel_;
	int goal_index_;

	ros::Subscriber point_sub_, odom_sub_;
	ros::Publisher goal_pub_, point_pub_, cmd_pub_;
	STATUS status_;
	std::vector<geometry_msgs::PoseStamped> goal_points_;
	double dist_tolerance_, goal_point_tolerance_;
	geometry_msgs::Pose current_pose_;
	int maximum_point_pub_size_;

	tf::TransformListener tf_;
	std::string robot_namespace_;
	std::string map_frame_name_;
	std::string body_link_name_;
};

} //  end of namespace

int main ( int argc, char **argv )
{
	ros::init ( argc, argv, "stair_climb" );
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	arti_navigation::StairClimb stair_climb(nh, private_nh);
	ros::spin();
	return 0;
}
