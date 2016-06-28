#pragma once
// http://fossies.org/linux/boost/libs/geometry/example/02_linestring_example.cpp 


// ENABLE ME FOR HIGHER PERFORMANCE ****************************************************##################
//#define NDEBUG
#include <iostream>
#include <cassert>
#include <iomanip>
#include <fstream>
#include <eigen3/Eigen/Dense>

namespace dad_local_planner
{
  

class ReferenceTrajectory
{
public:
std::vector<Eigen::Vector2d> points_;
std::vector<double> speeds_;
std::vector<double> cumulativeDistances_;
std::vector<double> segmentDistances_;
std::vector<double> segmentDistancesSq_;
std::vector<Eigen::Vector2d> segmentVectors_;
std::vector<Eigen::Vector2d> segmentUnitDirVectors_;
std::vector<Eigen::Vector2d> segmentUnitNormalVectors_;
ReferenceTrajectory()
{
}

ReferenceTrajectory(const ReferenceTrajectory& ref)
{
  points_ = ref.points_;
  speeds_ = ref.speeds_;
  cumulativeDistances_ = ref.cumulativeDistances_;
  segmentDistances_ = ref.segmentDistances_;
  segmentVectors_ = ref.segmentVectors_;
  segmentUnitDirVectors_ = ref.segmentUnitDirVectors_;
  segmentUnitNormalVectors_ = ref.segmentUnitNormalVectors_;
}

ReferenceTrajectory(const std::vector<Eigen::Vector2d>& pointList, const std::vector<double>& desiredSpeedList)
{
  assert(pointList.size() == desiredSpeedList.size() + 1 && " Sizes do not match ");
  
  points_ = pointList;
  speeds_ = desiredSpeedList;
  
  double cumulativeDistance = 0;
  for(int i = 1; i < points_.size() ; i++)
  {
    Eigen::Vector2d diff = points_[i] - points_[i-1];
    double segmentDist = diff.norm(); 
    assert( segmentDist > 0);
    segmentVectors_.push_back(diff);
    segmentDistances_.push_back(segmentDist);
    segmentDistancesSq_.push_back(segmentDist*segmentDist);
    assert(segmentDist > 0);
    segmentUnitDirVectors_.push_back(diff/segmentDist);
    
    Eigen::Vector2d normal = Eigen::Vector2d(-diff[1],diff[0]);
    segmentUnitNormalVectors_.push_back(normal/segmentDist);
    

    cumulativeDistance += segmentDist;  
    cumulativeDistances_.push_back(cumulativeDistance);
  }
} 
double getLastSegmentCumulativeDistance() const
{ 
  //double value = cumulativeDistances_[cumulativeDistances_.size()-1];
  assert(cumulativeDistances_.size()-2 >= 0 && " Ensure the given path have enough segments ");
  double value = cumulativeDistances_[cumulativeDistances_.size()-2];
  return value;
}
void getProjectionOnSegment(double& perpDistance, double& alongDistance, double& parameter,const Eigen::Vector2d& point, const int segId) const
{
  Eigen::Vector2d startPoint = points_[segId];
  Eigen::Vector2d posVector = point - startPoint;
  Eigen::Vector2d dirVector = segmentVectors_[segId];
  Eigen::Vector2d unitDirVector = segmentUnitDirVectors_[segId];
  double dirVectorNormSq = segmentDistancesSq_[segId];
  double dirVectorNorm = segmentDistances_[segId];
  double posVectorNorm = posVector.norm();
  
  parameter = posVector.dot(dirVector) / ( dirVectorNormSq );
  
  if ( parameter < 0 ) parameter = 0;
  if ( parameter > 1 ) parameter = 1;
  
  Eigen::Vector2d projPoint = startPoint + parameter * segmentVectors_[segId]; 
  
  
  alongDistance = parameter * dirVectorNorm; 
  perpDistance = ( point - projPoint ).norm();
  /*
  std::cout << "********" << std::endl;
  std::cout << "POS VEC : " << std::endl;
  std::cout << posVector << std::endl;
  std::cout << "POINT : " << std::endl;
  std::cout << point << std::endl;
  std::cout << "DIR VEC : " << std::endl;
  std::cout << dirVector << std::endl;
  std::cout << "PARAM : " << std::endl;
  std::cout << parameter << std::endl;
  std::cout << "DIR NORM: " << std::endl;
  std::cout << dirVectorNorm << std::endl;
  std::cout << "DIR NORM SQ: " << std::endl;
  std::cout << dirVectorNormSq << std::endl;
  std::cout << "PROJ PT : " << std::endl;
  std::cout << projPoint << std::endl;
  std::cout << "########" << std::endl;
  */
}

void getSegmentWindowFromScanWindow(int& minSegmentIndex, int& maxSegmentIndex, double minDist, const double maxDist) const
{
  minSegmentIndex = cumulativeDistances_.size() - 1;
  maxSegmentIndex = cumulativeDistances_.size() - 1;
  
  for(int i = 0; i < cumulativeDistances_.size() ; i++)
  {
    if ( cumulativeDistances_[i] > minDist )
    {
      minSegmentIndex = i;
      break;
    }

  }
  for(int i = 0; i < cumulativeDistances_.size() ; i++)
  {
    if ( cumulativeDistances_[i] > maxDist )
    {
      maxSegmentIndex = i;
      break;
    }

  }
}


void getCumulativeDistanceOnPath(double& cumulativeDistance, double& distanceFromPath, Eigen::Vector2d& referenceVelocity, const Eigen::Vector2d& point, const int minSegId, const int maxSegId) const
{
   double distanceOnLastSegment = 0;
   int stopIndex = maxSegId + 1;
   //std::cout << " Started " << std::endl;
    double perpDist = 0;
    double alongDist = 0;
    double parameter = 0;
  for ( int i = minSegId ; i <= maxSegId ; i++)
  {
    perpDist = 0;
    alongDist = 0;
    parameter = 0;

    getProjectionOnSegment(perpDist,alongDist,parameter,point,i);
    //std::cout << " PARAM : " << parameter << std::endl;
    //std::cout << " PERP : " << perpDist << std::endl;
    if ( parameter == 0 )
    {
      stopIndex = i;
      break;
    }
    distanceOnLastSegment = alongDist;
    distanceFromPath = perpDist;
    referenceVelocity = speeds_[i] * segmentUnitDirVectors_[i];
   // std::cout << " ALONG DIST : " << alongDist << std::endl;
  }
  //  std::cout << " STOP INDEX : " << stopIndex << std::endl;
  if ( stopIndex == 1)
  {
    cumulativeDistance = distanceOnLastSegment;
    referenceVelocity = speeds_[0] * segmentUnitDirVectors_[0];
  }
  else if ( stopIndex == 0 )
  {
    cumulativeDistance = distanceOnLastSegment;
    distanceFromPath = perpDist;
    referenceVelocity = speeds_[0] * segmentUnitDirVectors_[0];
  }
  /*
  if ( stopIndex < 2 )
  {
 //   std::cout << " STOP INDEX < 2 : " << distanceOnLastSegment << std::endl;
    cumulativeDistance = distanceOnLastSegment;
    if ( stopIndex < 1)
    distanceFromPath = perpDist;
  }
  */
  else
  {
//    std::cout << " STOP INDEX ow : " << cumulativeDistances_[stopIndex-2] << std::endl;
    cumulativeDistance = distanceOnLastSegment + cumulativeDistances_[stopIndex-2];
  }
}

/*
std::pair<double,int> getMinDistanceAndSegIndexWithinSegWindow(const Point& pt, const SegmentWindow& segWindow) const
{
    std::cout << " GET MIN DIST " << std::endl;
  int minSegmentIndex = std::get<SEG_WIN_MIN>(segWindow);
  int maxSegmentIndex = std::get<SEG_WIN_MAX>(segWindow);
  
    std::cout << " minSegId  " << minSegmentIndex <<" , maxSegId  " << maxSegmentIndex << std::endl;

  double result = std::numeric_limits< double >::max();
  int minLoc = 0;
  //std::cout << " ** " << std::endl;
  
  for(int k = minSegmentIndex; k <= maxSegmentIndex + 1 ; k++)
  {
  //double dist = boost::geometry::distance(pt, std::get<TS_LINE_SEG>(trajSegments_[k])); 
  double dist = boost::geometry::distance(pt, points_[k]) ; 
  std::cout << " Dist : " << dist << " , k = " << k << std::endl;
    if( result > dist)
    {
      result = dist;
      minLoc = k;
    }
  }
  minLoc--;
  //if ( minLoc < 0 ) minLoc = 0;
  std::pair<double,int> ret = std::make_pair(result,minLoc);
  return ret;
}
*/

void printTrajectory(const std::string& fname) const {
  std::ofstream file(fname);
  for(int k = 0; k < points_.size() ; k++ )
  { 
      file << points_[k][0] <<" "<< points_[k][1] << "\n";
  }
  file.close();

}


};

};