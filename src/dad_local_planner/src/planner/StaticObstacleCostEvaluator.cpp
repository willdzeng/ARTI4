#include "dad_local_planner/StaticObstacleCostEvaluator.h"

using namespace dad_local_planner;  
/**
 * Initializes the collision cost evaluator by making a copy of the costmap pointer
 */
void StaticObstacleCostEvaluator::initialize ( std::shared_ptr< costmap_2d::Costmap2DROS > costMapPtr, double staticObstacleCollisionRiskFactor )
    {
      // std::cout << " Collision cost evaluator constructor " << std::endl;
      costMapPtr_ = costMapPtr;
      staticObstacleCollisionRiskFactor_ = staticObstacleCollisionRiskFactor;
    }
/**
 * Populates transistion data related to collision costs. The transformed motion primitive is overlaid on the costmap.
 * Then, this motion primitive is scored by looking at the cost of the grids the motion primitive passes over. 
 */
void StaticObstacleCostEvaluator::populateTransitionData( Node& childNode, const Node& parentNode, const MotionPrimitive& transformedMp ) const
    {
        costmap_2d::Costmap2D* costmap = costMapPtr_->getCostmap();
	
	bool meaningful = true;
	double totalCost = 0;
	for(int i = 0; i < transformedMp.x.size() ; i++ )
	{
	    double x = transformedMp.x[i];
	    double y = transformedMp.y[i];
	    unsigned int cx = 0;
	    unsigned int cy = 0;
	    costmap->worldToMap(x,y,cx,cy);
	    
	    int cellValue = (int) costmap->getCost(cx,cy);
	    //std::cout << cellValue << std::endl;
	    if ( cellValue > 252 )
	    {
	      meaningful = false;
	      //std::cout << " BAD VAL " << std::endl;
	      break;
	    }
	    else
	    {
	      totalCost += cellValue;
	    }
	}
	
	if ( meaningful == true )
	{
	  totalCost /= transformedMp.x.size();
	  childNode.cCost = totalCost * staticObstacleCollisionRiskFactor_;
	}
	else
	{
	  childNode.cCost = 1000; // SEARCH_PERF
	  //std::cout << " BAD " << std::endl;
	}
      
    }
