#pragma once

#include <iostream>
#include <iomanip>
#include <queue>
#include <memory>
#include <fstream>


#include <unordered_map>

#include "Node.h"
#include "MotionPrimitive.h"
#include "MotionPrimitiveManager.h"
#include "TubeCostEvaluator.h"
#include "StaticObstacleCostEvaluator.h"
#include "SpatialHash.h"
#include "WaypointPath.h"
#include "Common.h"

#include <ros/ros.h>
#include <ros/publisher.h>
#include <std_msgs/String.h>

#include <rviz_debug_tools/RDT.h>
#include <dynamic_obstacle/DynamicObstacle.h>
#include <dad_local_planner/DynamicObstacleCostEvaluator.h>
#include <dad_local_planner/DADPlannerConfig.h>

namespace dad_local_planner
{
  
class AStarSearch
{
private:
protected:
    std::vector<Node> nodes_;
    std::priority_queue<Node, std::vector<Node>, NodeComparator> priorityQueue_;

    std::shared_ptr<WaypointPath> refTrajPtr_;
    std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr_;

    std::shared_ptr<SpatialHash> spatialHashPtr_;

    TubeCostEvaluator tubeCostEvaluator_;
    StaticObstacleCostEvaluator staticObstacleCostEvaluator_;
    
    DynamicObstacleCostEvaluator dynamicObstacleCostEvaluator_;

    std::shared_ptr<MotionPrimitiveManager> mpManagerPtr_;
    std::vector<int> mpIndexSet_;

    Node startNode_;
    Node goalNode_;

    bool searchStarted_;
    bool searchFinished_;
    int iterationCount_;

    // std::vector<Eigen::Vector2d> solution_;
    std::vector<Node> solution_;
    double bestCostSoFar_;


    
    ros::NodeHandle n_;
    rviz_debug_tools::RDT rdt;

    // parameters
    bool debug_;
    bool debugWaitInput_ ;
    int spatialHashSize_;
    double spatialHashTimeTolerance_;
    
    double detourCostFactor_;
    double skipCostFactor_;
    double staticObstacleCollisionRiskFactor_ ;
    double dynamicObstacleCollisionRiskFactor_;
    double tubeSize_;
    double subOptimalityEps_;
    
    double spatialConvergenceThreshold_;
    double angularConvergenceThreshold_;
    
    double robotRadius_;
    int trajResolution_;
    double accMaxFwd_;
    double accMaxBack_;
    // double velMax_;
    double velMaxFwd_;
    double velMaxBack_;


    bool canNodeBeSkipped(const Node &currentNode) const;
    
    void pushNodeIntoQueue(Node& childNode);
    
    bool populateChildNodeCosts(Node& childNode,const Node& parentNode, const MotionPrimitive& transformedMp);

    void addNeighbors(const Node &node);
    bool checkGoal(const Node &node) const;
    void traceBack(const Node &node);
    
    double lookAheadDistance_;
    
    std::vector<dynamic_obstacle::DynamicObstacle> dynamicObstaclesVector_;
    

    DADPlannerConfig config_;
public:
    // debug count
    int expansions_;
    int count_;
    
    AStarSearch();
    void initialize(
        std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr_, 
        std::shared_ptr<WaypointPath> refTrajPtr, 
        std::shared_ptr< std::vector<dynamic_obstacle::DynamicObstacle> > dynamicObstaclesVector, 
        const Node &startNode);
    bool search(int maxIterationCount);

    void setMotionPrimitiveManager(std::shared_ptr<MotionPrimitiveManager> motionPrimitiveManagerPtr);
    void setMotionPrimitiveIndexSet();
    void reconfigure(const DADPlannerConfig& config);

    // const std::vector<Eigen::Vector2d> &getSolution() const;
    const std::vector<Node>& getSolution() const;

    void setupViz(ros::NodeHandle nh) {
        n_ = nh;
        rdt.initialize(n_);
    }

};


};