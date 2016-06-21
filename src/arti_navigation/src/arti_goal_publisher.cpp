#include "arti_goal_publisher.h"
#include <string>
#include <vector>
#pragma once
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace arti_navigation {
class ArtiGoalPublisher {
public:
	enum STATUS { READY, CONTROLLING, DONE };
	ArtiGoalPublisher() {
		ros::NodeHandle nh();
		point_sub_ = nh.subscribe ( "clicked_point", 10, &ArtiGoalPublisher::onReceivePoint, this);
		goal_pub_ = nh.advertise("goal", geometry_msgs::PoseStamped, 1);
	}

	~ArtiGoalPublisher() {

	}

	onReceivePoint(const geometry_msgs::PointStamped::ConstPtr& msg) {
		ROS_INFO ( "Got point" );
		if ( status_ == ArtiGoalPublisher::STATUS::READY )
		{
			geometry_msgs::PointStamped point;
			point = &msg;
			bool add = false;
			if ( !points_.empty() )
			{
				// calcuate the euclidean distance
				double diff = (point.point.x - points_.back().point.x) ^ 2 + (point.point.y - points_.back().point.y) ^ 2;
				if ( diff > tolerance_ )
				{
					add = true;
				}
			}
			else
			{
				add = true;
			}

			//only add the point if it is not too close. this is a hack to overcome lack of UI support in rviz
			if ( add == true )
			{
				points_.push_back ( point );
			}
			else
			{
				status_ = ArtiGoalPublisher::STATUS::DONE;
			}
		}

		if (status_ == ArtiGoalPublisher::STATUS::DONE) {
			reset();
			points_.push_back(point);
		}
	}

	void reset()
	{
		points_.clear();
		status_ = IPP::STATUS::READY;
	}
private:
	ros::Subscriber point_sub_;
	ros::Publisher goal_pub_;
	STATUS status_;
	std::vector<geometry_msgs::PointStamped> points_;
}

}
