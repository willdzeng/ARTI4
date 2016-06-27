// #include "arti_goal_publisher.h"
// #pragma once
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

class MultiGoalPublisher {

public:
	enum STATUS { READY, CONTROLLING, DONE };
	MultiGoalPublisher(ros::NodeHandle nh, ros::NodeHandle private_nh) {
		nh_ = nh;
		point_sub_ = nh_.subscribe ("way_point", 10, &MultiGoalPublisher::pointCallback, this);
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		maximum_point_pub_size_ = 20;
		point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point", maximum_point_pub_size_);
		// odom_sub_ = nh.subscribe("odom", 1, &MultiGoalPublisher::odomCallback, this);
		private_nh.param("distance_tolerance", dist_tolerance_, 0.5);
		private_nh.param("robot_namespace", robot_namespace_, std::string(""));
		private_nh.param("map_frame_name", map_frame_name_, std::string("map"));
		private_nh.param("body_link_name", body_link_name_, std::string("base_link"));
		goal_point_tolerance_ = 0.2;
		reset();
	}

	~MultiGoalPublisher() {

	}

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		ROS_INFO_ONCE("Goal Publisher get odom");
		current_pose_ = msg->pose.pose;
	}

	void pointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
		// ROS_INFO ( "Got point" );
		geometry_msgs::PoseStamped point;
		point = *msg;

		if ( status_ == READY ) {
			bool add = false;
			if ( !points_.empty()) {
				// calcuate the euclidean distance
				double dist = distBetweenPose(point.pose, points_.back().pose);
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
				points_.push_back(point);
				// publish point on rviz;
				geometry_msgs::PointStamped ps;
				ps.point = point.pose.position;
				ps.header = point.header;
				point_pub_.publish(ps);
			}
			else {
				ROS_INFO("Finished points receiveing, goal size is %lu, start control", points_.size());
				status_ = CONTROLLING;
				controlling();
				return;
			}
		}

		if (status_ == CONTROLLING) {
			ROS_INFO("Cancel control and Wait new points publishing");
			reset();
			ROS_INFO("Add a new point %f %f %f", point.pose.position.x, point.pose.position.y, point.pose.orientation.z);
			points_.push_back(point);

		}
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


	void controlling() {
		ros::Rate r(10);
		int i = 0;
		while (nh_.ok() && i < points_.size() && status_ == CONTROLLING) {
			geometry_msgs::PoseStamped current_goal = points_[i];
			goal_pub_.publish(current_goal);
			bool next_goal = false;
			ROS_INFO("Wait for the robot moving");
			while (nh_.ok() && status_ == CONTROLLING && !next_goal) {
				updateRobotPose();
				double dist = distBetweenPose(current_goal.pose, current_pose_);
				ROS_INFO("Distance is %f", dist);
				if (dist < dist_tolerance_) {
					next_goal = true;
				}
				ros::spinOnce();
				r.sleep();
			}
			if (status_ != CONTROLLING) {
				break;
			}
			i++;
			ROS_INFO("Reached the goal, go to the next one, id: %d", i);
		}
		if (status_ != CONTROLLING) {
			ROS_INFO("Mission Canceled");
		} else {
			ROS_INFO("All goal reached, wait for new input");
			reset();
		}
		// reset();
	}

	double distBetweenPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
		return sqrt(pow((pose1.position.x - pose2.position.x), 2) +
		            pow((pose1.position.y - pose2.position.y), 2));
	}

	void reset()
	{
		points_.clear();
		status_ = READY;

		for (int i = 0 ; i < maximum_point_pub_size_; i++) {
			geometry_msgs::PointStamped empty_point;
			point_pub_.publish(empty_point);
		}
	}


private:
	ros::NodeHandle nh_;
	ros::Subscriber point_sub_, odom_sub_;
	ros::Publisher goal_pub_, point_pub_;
	STATUS status_;
	std::vector<geometry_msgs::PoseStamped> points_;
	double dist_tolerance_, goal_point_tolerance_;
	geometry_msgs::Pose current_pose_;
	int maximum_point_pub_size_;

	tf::TransformListener tf_;
	std::string robot_namespace_;
	std::string map_frame_name_;
	std::string body_link_name_;
};

} // end of namesapce


int main ( int argc, char **argv )
{
	ros::init ( argc, argv, "arti_goal_publisher" );
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	arti_navigation::MultiGoalPublisher arti_goal_publisher(nh, private_nh);
	ros::spin();
	return 0;
}