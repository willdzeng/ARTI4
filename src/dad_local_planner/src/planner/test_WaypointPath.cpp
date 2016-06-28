#include <dad_local_planner/WaypointPath.h>
using namespace dad_local_planner;

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_reference_trajectory

#include <boost/test/unit_test.hpp>
#include <boost/concept_check.hpp>
BOOST_AUTO_TEST_SUITE (waypoint_test) 
BOOST_AUTO_TEST_CASE( test_init )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,0));
     pts.push_back(Eigen::Vector2d(2,0));
     pts.push_back(Eigen::Vector2d(3,0));
     pts.push_back(Eigen::Vector2d(4,0));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     WaypointPath rf(pts,refSpeed); 
     Eigen::Vector2d p;
     double val = 0;

     BOOST_CHECK_EQUAL( rf.points_.size(), 5);
     BOOST_CHECK_EQUAL( rf.segmentDistances_.size(), 4);
     BOOST_CHECK_EQUAL( rf.speeds_.size(), 4);
}
BOOST_AUTO_TEST_CASE( test_getWindowIndex )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,0));
     pts.push_back(Eigen::Vector2d(2.5,0));
     pts.push_back(Eigen::Vector2d(3,0));
     pts.push_back(Eigen::Vector2d(4,0));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     WaypointPath rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 0;
     
     BOOST_TEST_MESSAGE("Testing if a full scan window is correctly converted to segment window ");
     rf.getWindowIndex(minSegmentIndex,maxSegmentIndex,0,4);
     BOOST_CHECK_EQUAL ( minSegmentIndex,0);
     BOOST_CHECK_EQUAL ( maxSegmentIndex,4);
}
BOOST_AUTO_TEST_CASE( test_getWindowIndex2 )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,0));
     pts.push_back(Eigen::Vector2d(2.5,0));
     pts.push_back(Eigen::Vector2d(3,0));
     pts.push_back(Eigen::Vector2d(4,0));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     WaypointPath rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 0;
     
     BOOST_TEST_MESSAGE("Testing if a partial scan window is correctly converted to segment window ");
     rf.getWindowIndex(minSegmentIndex,maxSegmentIndex,2,4);
     BOOST_CHECK_EQUAL ( minSegmentIndex,2);
     BOOST_CHECK_EQUAL ( maxSegmentIndex,4);
}
BOOST_AUTO_TEST_CASE( test_getWindowIndex3 )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,1));
     pts.push_back(Eigen::Vector2d(2,2));
     pts.push_back(Eigen::Vector2d(3,3));
     pts.push_back(Eigen::Vector2d(4,4));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     WaypointPath rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 0;
     
     BOOST_TEST_MESSAGE("Testing if a partial scan window exceeding the limits of the given point list is correctly converted to segment window ");
     rf.getWindowIndex(minSegmentIndex,maxSegmentIndex,2,8);
     BOOST_CHECK_EQUAL ( minSegmentIndex,2);
     BOOST_CHECK_EQUAL ( maxSegmentIndex,4);
}
BOOST_AUTO_TEST_CASE( test_getDistanceData)
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,0));
     pts.push_back(Eigen::Vector2d(2,0));
     pts.push_back(Eigen::Vector2d(3,0));
     pts.push_back(Eigen::Vector2d(4,0));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     WaypointPath rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 0;
     
     BOOST_TEST_MESSAGE("Testing if distance data is correctly calculated for simple cases");
     rf.getWindowIndex(minSegmentIndex,maxSegmentIndex,0,8);
     
     double pathPosition;
     double distFromPath;
     
     Eigen::Vector2d refVel;
     Eigen::Vector2d point;
     point[0] = 2;
     point[1] = 2;
     rf.getDistanceData(pathPosition,distFromPath,refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(pathPosition - 2) < 0.01 );
     BOOST_CHECK( std::abs(distFromPath - 2) < 0.01 );
     
     point[0] = 3;
     point[1] = 2;
     rf.getDistanceData(pathPosition,distFromPath,refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(pathPosition - 3) < 0.01 );
     BOOST_CHECK( std::abs(distFromPath - 2) < 0.01 );
     
     point[0] = 3;
     point[1] = 1;
     rf.getDistanceData(pathPosition,distFromPath,refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(pathPosition - 3) < 0.01 );
     BOOST_CHECK( std::abs(distFromPath - 1) < 0.01 );
     
     point[0] = 3.5;
     point[1] = 1;
     rf.getDistanceData(pathPosition,distFromPath,refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(pathPosition - 3) < 0.01 );
     BOOST_CHECK( std::abs(distFromPath - std::sqrt(0.5*0.5 + 1*1) )< 0.01 );
}

BOOST_AUTO_TEST_SUITE_END( )