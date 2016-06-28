#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <string>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <dad_local_planner/DADPlannerROSConfig.h>
#include <dad_local_planner/DADPlannerConfig.h>


#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ugv_msgs/LocalPlan.h>
#include <assert.h>
#include <dad_local_planner/dad_planner.h>
#include <ugv_msgs/DynamicObstacles.h>

namespace dad_local_planner {
  class DADPlannerROS{
    private:

      bool initialized_;
      bool setup_;

      std::shared_ptr<DADPlanner> dp_;
      
      std::string globalFrame_;
      std::string dynamicObstacleTopic_;
      std::string referencePathTopic_;
      
      ros::Subscriber globalPlanSub_;
      ros::Subscriber dynamicObstacleSub_;
      ros::Subscriber odometrySub_;
      
      double currentPositionX_;
      double currentPositionY_;
      double currentHeading_;
      
      ros::Publisher localPlanPathPub_;
      ros::Publisher localPlanTrajPub_;

      std::shared_ptr<costmap_2d::Costmap2DROS> staticCostMap_;
      std::shared_ptr<std::vector<dynamic_obstacle::DynamicObstacle> > dynamicObstaclesVectorPtr_;
      
      std::shared_ptr<tf::TransformListener> tf_; 
      std::string odomTopic_;

      std::string mpPath_;
      

      dynamic_reconfigure::Server<DADPlannerROSConfig> *dsrv_;
      dad_local_planner::DADPlannerROSConfig defaultConfig_;

      DADPlannerConfig config_;

      bool got_obstacle_;

      ros::Time startTime_;
      
      void reconfigureCB(DADPlannerROSConfig &config, uint32_t level);
      void publishLocalPlan();
      
      void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg);
      void dynamicObstaclesCallback(const ugv_msgs::DynamicObstacles::ConstPtr& obstacles);
      void odometryCallback(const nav_msgs::Odometry::ConstPtr& poseStamped);
    public:
      DADPlannerROS();
      void initialize(std::string name
          );
      ~DADPlannerROS();
      bool setReferenceTrajectory(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      void plan();
      bool isInitialized() {
        return initialized_;
      }
      
      void setupViz(ros::NodeHandle nh)
      {
        dp_->setupViz(nh);
      }



  };
};
