#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <cassert>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>

  class PurePursuitController {
  public:
    enum STATUS { INITIALIZED, READY, EXECUTING, DONE };
    /// Constructor
    PurePursuitController(const ros::NodeHandle& nh);
    ~PurePursuitController();
    /// Step once
    bool getNextCmdVel(geometry_msgs::Twist& twist);
    /// Returns the current pose of the robot
    geometry_msgs::PoseStamped getCurrentPose() const;
    /// Returns the lookahead distance for the given pose
    double getLookAheadDistance(const geometry_msgs::PoseStamped& pose) const;
    /// Returns the lookahead angle for the given pose in [rad]
    double getLookAheadAngle(const geometry_msgs::PoseStamped& pose) const;
    /// Returns the current lookahead distance threshold
    double getLookAheadThreshold() const;
    /// Returns the lookahead distance for the given pose
    double getArcDistance(const geometry_msgs::PoseStamped& pose) const;
    /// Returns the next way point by linear search from the current waypoint
    int getNextWayPoint(int wayPoint) const;    
    /// Returns the current closest waypoint
    int getClosestWayPoint() const;    
    /// Returns the interpolated pose based on the given way point
    bool getInterpolatedPose(int wayPoint, geometry_msgs::PoseStamped& interpolated_pose) const;
    void transformToWorld(const geometry_msgs::PoseStamped& input_pose, geometry_msgs::PoseStamped& output_pose) const;
    void reset();

    /// Retrieves parameters
    void getParameters();
    /// Path message callback
    void pathCallback(const nav_msgs::Path& msg);
    /// Odometry message callback
    void odometryCallback(const nav_msgs::Odometry& msg);
    /// Timer callback
    void timerCallback(const ros::TimerEvent& event);

private:
    /// ROS node handle
    ros::NodeHandle nh_;
    /// ROS subscriber
    ros::Subscriber path_sub_, odom_sub_;
    /// ROS publisher
    ros::Publisher cmd_vel_pub_, cmd_traj_pub_, goal_point_pub_, heading_vecotr_pub_, look_ahead_pub_;

    /// Topic and frame name
    std::string path_topic_name_;
    std::string odom_topic_name_;
    std::string pose_frame_id_;
    std::string cmd_vel_topic_;
    std::string cmd_traj_topic_;
    std::string ref_path_frame_id_;
    std::string robot_namespace_;
    std::string odom_frame_id_;
    std::string body_frame_id_;

    /// Queue size for receiving messages
    int queue_size_;
    /// Current reference path
    nav_msgs::Path cur_ref_path_;
    /// Current velocity
    geometry_msgs::Twist cur_vel_;
    tf::StampedTransform cur_pose_;
    /// Controller frequency
    double frequency_;
    /// Next way point
    int next_waypoint_;

    /// Initial way point
    int initial_waypoint_;
    /// Velocity
    double target_velocity_;
    /// Lookahead ratio
    double lookahead_ratio_;
    /// Epsilon
    double epsilon_;
    /// Transform listener for robot's pose w.r.t. map
    tf::TransformListener tf_listenner_;
    /// Timer
    ros::Timer timer_;
    
    STATUS status_;

    nav_msgs::Odometry cur_odom_;

    // position and angle
    double Px_,Py_,Pz_,theta_x_,theta_y_,theta_z_;
    // velocity
    double Vx_, Vy_, Vz_, omega_x_, omega_y_, omega_z_;
  };
