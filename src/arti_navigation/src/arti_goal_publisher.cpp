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

namespace arti_navigation {

class ArtiGoalPublisher {

public:
	enum STATUS { READY, CONTROLLING, DONE };
	ArtiGoalPublisher(ros::NodeHandle nh, ros::NodeHandle private_nh) {
		nh_ = nh;
		point_sub_ = nh_.subscribe ("way_point", 10, &ArtiGoalPublisher::pointCallback, this);
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		maximum_point_pub_size_ = 20;
		point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point", maximum_point_pub_size_);
		odom_sub_ = nh.subscribe("odom", 1, &ArtiGoalPublisher::odomCallback, this);
		private_nh.param("distance_tolerance", dist_tolerance_, 0.5);
		goal_point_tolerance_ = 0.2;
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
			ROS_INFO("Wait for the robot moving");
			while (nh_.ok() && status_ == CONTROLLING && !next_goal) {
				double dist = distBetweenPose(current_goal.pose, current_pose_); //current_goal.pose.position.x - current_pose_.position.x
				ROS_INFO("Distance is %f",dist);
				if (dist < dist_tolerance_) {
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

		for (int i = 0 ; i < maximum_point_pub_size_; i++){
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
};

} // end of namesapce


int main ( int argc, char **argv )
{
	ros::init ( argc, argv, "arti_goal_publisher" );
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	arti_navigation::ArtiGoalPublisher arti_goal_publisher(nh,private_nh);
	ros::spin();
	return 0;
}