#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>


  /** The class PurePursuitControllerNode implements a pure pursuit controller.
      \brief Pure pursuit controller
    */
  class PurePursuitController {
  public:
    enum STATUS { INITIALIZED, READY, EXECUTING, DONE };
    /// Constructor
    PurePursuitController(const ros::NodeHandle& nh);
    /// Copy constructor
    PurePursuitController(const PurePursuitController& other) = delete;
    /// Copy assignment operator
    PurePursuitController& operator = (const PurePursuitController& other) = delete;
    /// Move constructor
    PurePursuitController(PurePursuitController&& other) = delete;
    /// Move assignment operator
    PurePursuitController& operator = (PurePursuitController&& other) = delete;
    /// Destructor
    virtual ~PurePursuitController();
    /// Spin once
    void spin();
    /// Step once
    bool getCommandVelocity(geometry_msgs::Twist& twist);
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

  protected:
    /** \name Protected methods
      @{
      */
    /// Retrieves parameters
    void getParameters();
    /// Path message callback
    void pathCallback(const nav_msgs::Path& msg);
    /// Odometry message callback
    void odometryCallback(const nav_msgs::Odometry& msg);
    /// Timer callback
    void timerCallback(const ros::TimerEvent& event);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle nh_;
    /// Path message subscriber
    ros::Subscriber path_sub_;
    /// Path message topic name
    std::string path_topic_name_;
    /// Odometry message subscriber
    ros::Subscriber odom_sub_;
    /// Odometry message topic name
    std::string odom_topic_name_;
    /// Frame id of pose estimates
    std::string pose_frame_id_;

    std::string ref_path_frame_id_;
    /// Queue size for receiving messages
    int queue_size_;
    /// Current reference path
    nav_msgs::Path cur_ref_path_;
    /// Current velocity
    geometry_msgs::Twist cur_vel_;
    /// Controller frequency
    double frequency_;
    /// Next way point
    int next_waypoint_;
    /// Commanded velocity publisher
    ros::Publisher cmd_vel_pub_;
    /// Commanded velocity topic name
    std::string cmd_vel_topic_;
    /// Commanded trajectory publisher
    ros::Publisher cmd_traj_pub_;
    /// Commanded trajectory topic name
    std::string cmd_traj_topic_;

    ros::Publisher goal_point_pub_;
    ros::Publisher heading_vecotr_pub_;
    ros::Publisher look_ahead_pub_;
    
    /// Initial way point
    int initial_waypoint_;
    /// Velocity
    double velocity_;
    /// Lookahead ratio
    double lookahead_ratio_;
    /// Epsilon
    double epsilon_;
    /// Transform listener for robot's pose w.r.t. map
    tf::TransformListener tf_listenner_;
    /// Timer
    ros::Timer timer_;
    
    STATUS status_;

  };
