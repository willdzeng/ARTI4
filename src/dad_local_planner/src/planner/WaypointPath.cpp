#include <dad_local_planner/WaypointPath.h>

using namespace dad_local_planner;

WaypointPath::WaypointPath()
{

}
WaypointPath::WaypointPath(const std::vector< Eigen::Vector2d >& pointList, const std::vector< double >& desiredSpeedList)
{
    assert(pointList.size() == desiredSpeedList.size() && " Sizes do not match ");

    points_ = pointList;
    speeds_ = desiredSpeedList;

    double cumulativeDistance = 0;
    for (int i = 1; i < points_.size() ; i++)
    {
        Eigen::Vector2d diff = points_[i] - points_[i - 1];
        double segmentDist = diff.norm();
        //assert( segmentDist > 0);
        segmentDistances_.push_back(segmentDist);
        segmentDistancesSq_.push_back(segmentDist * segmentDist);
	
        cumulativeDistances_.push_back(cumulativeDistance);
        cumulativeDistance += segmentDist;
	
	//std::cout << "\n Added point from " << points_[i][0] << "," << points_[i][1] << " to " << points_[i-1][0] << " , " << points_[i-1][1] << std::endl;
	//std::cout << "\n Cum dist : " << cumulativeDistances_[i-1] << std::endl;
    }
}


const Eigen::Vector2d& WaypointPath::getPoint(int index) const
{
    return points_[index];
}

const Eigen::Vector2d& WaypointPath::getLastPoint() const
{
  return points_.back();
}


double WaypointPath::getSpeed(int index) const
{
    return speeds_[index];
}
double WaypointPath::getCumulativeDistance(int index) const
{
    return cumulativeDistances_[index];
}
double WaypointPath::getLastCumulativeDistance() const
{
  return cumulativeDistances_.back();
}

void WaypointPath::getWindowIndex(int& minIndex, int& maxIndex, const double minDist, const double maxDist) const
{
    minIndex = cumulativeDistances_.size() - 1;
    maxIndex = cumulativeDistances_.size() - 1;

    for ( int i = 0 ; i < cumulativeDistances_.size() ; i++ )
    {
        minIndex = i;
        if ( cumulativeDistances_[i] >= minDist )
        {
            break;
        }
    }
    for ( int i = cumulativeDistances_.size() ; i >= 0 ; i-- )
    {
        maxIndex = i;
        if ( cumulativeDistances_[i] <= maxDist )
        {
            break;
        }
    }

}
void WaypointPath::getDistanceData(double& cumulativeDistance, double& distanceFromPath, Eigen::Vector2d& referenceVelocity, const Eigen::Vector2d& point, const int minIndex, const int maxIndex) const
{
    double distanceOnLastSegment = 0;
    int stopIndex = maxIndex + 1;
    
    double lowestDistance = std::numeric_limits<double>::max();
    int minId = -1;
    for ( int i = minIndex ; i <= maxIndex ; i++)
    {
      double dist = ( point - points_[i] ).norm();
      if ( lowestDistance >  dist )
      {
	  lowestDistance = dist; 
	  minId = i;
      }
    }
    
    cumulativeDistance = cumulativeDistances_[minId];
    distanceFromPath = lowestDistance;
    
    referenceVelocity[0] = speeds_[minId];
    referenceVelocity[1] = -100;
}
