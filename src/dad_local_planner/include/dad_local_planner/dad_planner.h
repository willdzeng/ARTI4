#pragma once

#include <vector>
#include <Eigen/Core>
#include <assert.h>

#include <dad_local_planner/DADPlannerConfig.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <nav_msgs/Path.h>

#include <ostream>

#include <cmath>
#include <queue>
#include <memory>

#include <angles/angles.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <dad_local_planner/ReferenceTrajectory.h>
#include <dad_local_planner/Node.h>
#include <dad_local_planner/AStarSearch.h>
#include <dad_local_planner/MotionPrimitiveManager.h>
#include <dad_local_planner/Trajectory.h>
#include <dynamic_obstacle/DynamicObstacle.h>

#include <boost/timer.hpp>


#define LINEAR_DIST_THRESH (0.5)
#define ANGULAR_DIST_THRESH (0.5)

namespace dad_local_planner
{

class DADPlanner
{
    enum STATUS { INITIALIZED, READY , PLANNING, DONE };
private:

    std::vector<geometry_msgs::PoseStamped> points_;
    std::shared_ptr<WaypointPath> refTraj_;

    AStarSearch aSearch_;
    boost::timer searchTimer_;
    std::shared_ptr<MotionPrimitiveManager> motionPrimitiveManagerPtr_;
    std::shared_ptr<std::vector<dynamic_obstacle::DynamicObstacle> > dynamicObstaclesVectorPtr_;
    std::shared_ptr<DynamicsConstraints> dynamicsConstraints_;
    // std::shared_ptr<std::vector<dynamic_obstacle::DynamicObstacle>> dynamicObstaclesVector_;
    
    
    std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr_;

    boost::mutex configuration_mutex_;
    
    STATUS status_;

    int trajResolution_;
    double mpDuration_;
    int maxIterationCount_;
    std::string mpPath_;
    std::string globalFrameId_;
    DADPlannerConfig config_;
    double accMaxFwd_;
    double accMaxBack_;
    double velMaxFwd_;
    double velMaxBack_;
    double curvatureMax_;
public:
    STATUS getPlannerStatus();
    
    DADPlanner(std::string name , std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr, std::string mpPath);
    ~DADPlanner() {

    }
    bool loadMotionPrimitives(std::string fileName);
    void reconfigure(DADPlannerConfig& cfg);
    
    bool plan();
    
    void getSolution(std::vector<Eigen::Vector3d> &postionList, std::vector<double> &timeList) const;

    bool setReferenceTrajectory(const std::vector<geometry_msgs::PoseStamped>& referenceTrajectory);

    void setDynamicObstacles(std::shared_ptr< std::vector<dynamic_obstacle::DynamicObstacle> > dynamicObstaclesVectorPtr );

    void startSearch(double x, double y, double th);

    void setupViz(ros::NodeHandle nh)
    {
      aSearch_.setupViz(nh);
    }
};

};
