// #include "arti_goal_publisher.h"
// #pragma once
#include <string>
#include <vector>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
// #include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

namespace arti_navigation {

class ArtiGoalPublisher {

public:
	enum STATUS { READY, CONTROLLING, DONE };
	ArtiGoalPublisher(ros::NodeHandle nh) {
		nh_ = nh;
		point_sub_ = nh_.subscribe ("way_point", 10, &ArtiGoalPublisher::pointCallback, this);
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		odom_sub_ = nh.subscribe("odom", 1, &ArtiGoalPublisher::odomCallback, this);
		ros::NodeHandle priviate_nh("~");
		tolerance_ = 0.2;
		reset();
	}

	~ArtiGoalPublisher() {

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
				if ( dist > 0.2 ) {
					add = true;
				}
			}
			else{
				add = true;
			}
			//only add the point if it is not too close. this is a hack to overcome lack of UI support in rviz
			if ( add == true ) {
				ROS_INFO("Add a new point %f %f %f", point.pose.position.x, point.pose.position.y, point.pose.orientation.z);
				points_.push_back(point);
			}
			else{
				ROS_INFO("Finished points receiveing, start control");
				status_ = CONTROLLING;
				controlling();
			}
		}

		if (status_ == CONTROLLING) {
			ROS_INFO("Cancel control and Wait new points publishing");
			reset();
			points_.push_back(point);
		}
	}

	void controlling() {
		ros::Rate r(10);
		int i = 0;
		while (nh_.ok() && i < points_.size() && status_ == CONTROLLING) {
			geometry_msgs::PoseStamped current_goal = points_[i];
			goal_pub_.publish(current_goal);
			bool next_goal = false;
			while (nh_.ok() && status_ == CONTROLLING && !next_goal) {
				double dist = distBetweenPose(current_goal.pose, current_pose_); //current_goal.pose.position.x - current_pose_.position.x
				if (dist < tolerance_) {
					next_goal = true;
				}
				ros::spinOnce();
				r.sleep();
			}
			i++;
			ROS_INFO("Reached the goal, go to the next one, id: %d", i);
		}

		ROS_INFO("All goal reached, wait for new input");
		status_ = READY;
		reset();
	}

	double distBetweenPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
		return sqrt(pow((pose1.position.x - pose2.position.x), 2) +
		       pow((pose1.position.y - pose2.position.y), 2));
	}

	void reset()
	{
		points_.clear();
		status_ = READY;
	}


private:
	ros::NodeHandle nh_;
	ros::Subscriber point_sub_, odom_sub_;
	ros::Publisher goal_pub_;
	STATUS status_;
	std::vector<geometry_msgs::PoseStamped> points_;
	double tolerance_;
	geometry_msgs::Pose current_pose_;
};

} // end of namesapce


int main ( int argc, char **argv )
{
	ros::init ( argc, argv, "arti_goal_publisher" );
	ros::NodeHandle nh;
	arti_navigation::ArtiGoalPublisher arti_goal_publisher(nh);
	ros::spin();
	return 0;
}