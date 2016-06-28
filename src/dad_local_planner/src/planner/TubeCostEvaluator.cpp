#include <dad_local_planner/TubeCostEvaluator.h>


using namespace dad_local_planner;

void TubeCostEvaluator::initialize(
  std::shared_ptr<WaypointPath> refTrajPtr,
  const double lookAhead,
  const double detourCostFactor,
  const double tubeSize,
  const double skipCostFactor)
{
  refTrajPtr_ = refTrajPtr;
  lookAhead_ = lookAhead;
  detourCostFactor_ = detourCostFactor;
  tubeSize_ = tubeSize;
  skipCostFactor_ = skipCostFactor;
  printf("cost evaluator got parameters: \n");
  printf("                              look ahead distance: %f\n", lookAhead_);
  printf("                              detourCostFactor_:   %f\n", detourCostFactor_);
  printf("                              tubeSize_:           %f\n", tubeSize_);
  printf("                              skipCostFactor_:     %f\n", skipCostFactor_);
}
/**
 * This function evaluates the cost of transition from the parentNode to the childNode. A scan window into the reference trajectory
 * is first computed. Then, based on the window, the average distance of the transformed motion primitive is calculated. This distance
 * and the velocity ( in this case the path position difference ) is used to determine if the childNode is a follow node or detour node.
 * @param[in,out] childNode The child node to be populated by this function
 * @param[in] parentNode The parent node to be used in the state machine
 * @param[in] transformedMp The motion primitive that has been transformed with start point at parentNode and end point at childNode
 */
void TubeCostEvaluator::populateTransitionData(Node& childNode, const Node& parentNode, const MotionPrimitive& transformedMp) const
{
  double duration = childNode.t - parentNode.t;
  // Compute transition cost
  int minIndex;
  int maxIndex;
  refTrajPtr_->getWindowIndex(minIndex,maxIndex,parentNode.pathPosition,parentNode.pathPosition + lookAhead_);// SEARCH_PERF
  //refTrajPtr_->getSegmentWindowFromScanWindow(minSegId, maxSegId, parentNode.pathPosition , parentNode.pathPosition + lookAhead_); 

  double distFromPath = 0;
  double avgDistFromPath = 0;
  double pathPosition = 0;
  double refVelocity = 0;
  Eigen::Vector2d refVel;

  //SEARCH_PERF
/*
  for (int m = 0; m < transformedMp.x.size() ; m++)
  {
    Eigen::Vector2d newPoint = Eigen::Vector2d(transformedMp.x[m], transformedMp.y[m]);
    //refTrajPtr_->getCumulativeDistanceOnPath(pathPosition, distFromPath, refVel, newPoint, minSegId, maxSegId);
    refTrajPtr_->getDistanceData(pathPosition,distFromPath,refVel,newPoint,minIndex,maxIndex);
    avgDistFromPath += distFromPath;
  }
  refVelocity = refVel[0];
  avgDistFromPath = avgDistFromPath / transformedMp.x.size();
  distFromPath = avgDistFromPath;
  childNode.avgDistFromPath = avgDistFromPath;
  */





  Eigen::Vector2d newPoint = Eigen::Vector2d(transformedMp.x.back(),transformedMp.y.back());
  refTrajPtr_->getDistanceData(pathPosition,distFromPath,refVel,newPoint,minIndex,maxIndex);
  
  refVelocity = refVel[0];
  avgDistFromPath = distFromPath;//transformedMp.x.size();
  childNode.avgDistFromPath = avgDistFromPath;



  /*
  std::cout << " REFERENCE VEL : " << std::endl;
  std::cout << refVel << std::endl;
  std::cout << " AVG DIST FRM PATH : " << std::endl;
  std::cout << avgDistFromPath << std::endl;
  */

  childNode.pathPosition = pathPosition;

  //Eigen::Vector2d dir = Eigen::Vector2d(childNode.x - parentNode.x, childNode.y - parentNode.y);
  Eigen::Vector2d dir = Eigen::Vector2d(std::cos(childNode.th), std::sin(childNode.th));
  

  //std::cout << " Now looking at end point : " << std::endl;
  //std::cout << newPoint << std::endl;
  //std::cout << " Distance from path : " << distFromPath << std::endl;
  
  /*
  double xx = refVel[0];
  double yy = refVel[1];
  */
  
  double x1 = dir[0];
  double y1 = dir[1];

  double progress = childNode.pathPosition - parentNode.pathPosition;
  double velocityError = std::abs(refVelocity - childNode.v);

  switch (parentNode.state)
  {
  case Node::STATE::FOLLOW:
    //if ( distFromPath < tubeSize && dir.dot(refVel) > 0) // be sure to change it in the detour case too
    /*
     * progress here makes sure that forward motion along path is encouraged. dir dot refvel ensures that direction of velocity is correct
     */
    if ( distFromPath < tubeSize_ && progress > 0 && velocityError < 0.2 ) // && dir.dot(refVel) > 0) // be sure to change it in the detour case too
    {
      childNode.state = Node::STATE::FOLLOW;
      double factor = avgDistFromPath / tubeSize_;

      childNode.pathPositionAtStartDeviation = -1;
      childNode.pathPositionAtEndDeviation = -1;
      childNode.dCost = 0;
      childNode.tCost = 0;//MAX_DURATION_ONE_MP *  ( (factor > 1) ? 1 : factor );

    }
    else
    {
      childNode.state = Node::STATE::DETOUR;
      childNode.pathPositionAtStartDeviation = parentNode.pathPosition;
      childNode.pathPositionAtEndDeviation = -1;
      childNode.dCost = 0;
      childNode.tCost = duration;
    }
    break;
  case Node::STATE::DETOUR:
    //if ( distFromPath > tubeSize || dir.dot(refVel) < 0 ) // be sure to change it in the follow case too
    if ( distFromPath > tubeSize_ || progress < 0 || velocityError > 0.2) // || dir.dot(refVel) < 0) // be sure to change it in the follow case too
    {
      childNode.state = Node::STATE::DETOUR;
      childNode.pathPositionAtStartDeviation = parentNode.pathPositionAtStartDeviation;
      childNode.pathPositionAtEndDeviation = -1;
      //childNode.skipCost = skipCostMultiplier * (MAX_DURATION_ONE_MP/MAX_DISTANCE_ONE_MP) * std::abs( childNode.pathPosition - childNode.pathPositionAtStartDeviation );
      childNode.dCost = 0;
      childNode.tCost = duration;
    }
    else
    {

      //std::cout << " childNode " << childNode.id << " has parentNode " << parentNode.id << std::endl;
      childNode.state = Node::STATE::FOLLOW;
      childNode.pathPositionAtStartDeviation = parentNode.pathPositionAtStartDeviation;
      childNode.pathPositionAtEndDeviation = childNode.pathPosition; // Current position corresponds to the end of deviation
      childNode.dCost = skipCostFactor_ * duration * std::abs( childNode.pathPositionAtEndDeviation - childNode.pathPositionAtStartDeviation );
      //childNode.tCost = childNode.skipCost + MAX_DURATION_ONE_MP;
      childNode.tCost = duration;
      /*
      std::cout << " ^^^^^^^^^^^^^^****************^^^^^^^^^^^^^^^^" << std::endl;
      std::cout << childNode << std::endl;
      std::cout <<  std::endl;
      std::cout << " From ( " << parentNode.x << " , " << parentNode.y << " ) ---> ( " << childNode.x << " , " << childNode.y << " ) "  << std::endl;
      std::cout << " ^^^^^^^^^^^^^^******#############**********^^^^^^^^^^^^^^^^" << std::endl;
      */
      childNode.pathPositionAtStartDeviation = -1;
      childNode.pathPositionAtEndDeviation = -1;
      /*
       std::cout << std::endl;
       std::cout << " Applying skip cost " << std::endl;
       std::cout << " Skip cost : " << childNode.skipCost << std::endl;
       std::cout << " PosStart : " << childNode.pathPositionAtStartDeviation << std::endl;
       std::cout << " PosEnd : " << childNode.pathPositionAtEndDeviation << std::endl;
       std::cout << std::endl;
       std::cout << childNode << std::endl;
       std::cout << std::endl;
       //std::cin.get();
       */


    }
    break;
  }

}
