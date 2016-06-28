#pragma once
#include <limits>
#include <cassert>
#include <costmap_2d/costmap_2d_ros.h>
#include "dad_local_planner/Node.h"
#include "dad_local_planner/MotionPrimitive.h"

namespace dad_local_planner
{
  
class StaticObstacleCostEvaluator
{
private:
  std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr_;
  
  double staticObstacleCollisionRiskFactor_;
public:
  StaticObstacleCostEvaluator(){};
  void initialize(std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr, double staticObstacleCollisionRiskFactor);
  void populateTransitionData(Node& childNode,const Node& parentNode, const MotionPrimitive& mp) const;

};

};