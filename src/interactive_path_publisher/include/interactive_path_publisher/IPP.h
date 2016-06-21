#pragma once

#include <iostream>
#include <vector>


#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>

class IPP
{
public:
  enum STATUS { READY, DONE };
  IPP ( ros::NodeHandle nh );
  void publishPath();
  void onReceivePoint ( const geometry_msgs::PointStamped::ConstPtr& msg );
  STATUS getStatus() const;
  void reset();

private:
  ros::NodeHandle nh_;
  ros::Subscriber pointSub_;
  ros::Publisher pathPub_;
  std::vector<Eigen::Vector2d> points_;

  double tolerance_;

  STATUS status_;
};
