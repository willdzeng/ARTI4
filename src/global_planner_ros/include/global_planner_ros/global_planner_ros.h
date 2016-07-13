
#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_
#define GLOBAL_PLANNER_GLOBAL_PLANNER_

#include <vector>
#include <string>

#include <ros/ros.h>

// #include <actionlib/server/simple_action_server.h>

// #include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "global_planner_ros/GlobalPlannerROSConfig.h"
#include <base_local_planner/goal_functions.h>
#include <std_msgs/Bool.h>
#include <global_planner/planner_core.h>

namespace global_planner_ros {
  //typedefs to help us out with the action server so that we don't hace to type so much

  enum GlobalPlannerState {
    PLANNING,
    CONTROLLING,
    CLEARING,
    WAITTING
  };

  /**
   * @class GlobalPlannerROS
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class GlobalPlannerROS {

    public:
        GlobalPlannerROS(tf::TransformListener& tf);
        void reconfigureCB(global_planner_ros::GlobalPlannerROSConfig &config, uint32_t level);
        void planningCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal);
        void replan(const geometry_msgs::PoseStamped goal);
        bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
        ~GlobalPlannerROS();
        bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        bool isQuaternionValid(const geometry_msgs::Quaternion& q);
        geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
        void planCollisionCheck();
        void smoothPath(std::vector<geometry_msgs::PoseStamped> old_path, std::vector<geometry_msgs::PoseStamped>& new_path);
        bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);
        bool getRobotPose(geometry_msgs::PoseStamped& pose);

    private:
      tf::TransformListener& tf_;

      costmap_2d::Costmap2DROS* planner_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      boost::thread* collision_check_thread_;
      ros::Publisher  plan_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      geometry_msgs::PoseStamped global_goal_;
      geometry_msgs::PoseStamped::ConstPtr global_goal_ptr_;

      GlobalPlannerState state_;

      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

      //set up plan triple buffer.
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* new_planner_plan_;
      bool new_plan_;

      //dynamic reconfigure.
      dynamic_reconfigure::Server<global_planner_ros::GlobalPlannerROSConfig> *dsrv_;
      std::string robot_namespace_,global_frame_,robot_frame_;

      //collision_check thread.
      boost::thread* ollision_check_;
      double collision_check_frequency_;

      //smoothpath
      double smooth_tolerance_, smooth_delta_, smooth_weight_;
  };
};
#endif

