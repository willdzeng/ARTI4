#pragma once

#include "Node.h"
#include "MotionPrimitive.h"
#include "WaypointPath.h"

#include <memory>


namespace dad_local_planner
{

class TubeCostEvaluator
{
private:
    std::shared_ptr<WaypointPath> refTrajPtr_;
    double lookAhead_;
    double detourCostFactor_;
    double maxDistanceOneMP_;
    double tubeSize_;
    double skipCostFactor_;
public:
    TubeCostEvaluator() {};
	void initialize(
      std::shared_ptr<WaypointPath> refTrajPtr,
      const double lookAhead,
      const double detourCostFactor,
      const double tubeSize,
      const double skipCostFactor);
     void populateTransitionData(Node &childNode, const Node &parentNode, const MotionPrimitive &mp) const;
};

};
