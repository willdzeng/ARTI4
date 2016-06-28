#include <dad_local_planner/ReferenceTrajectory.h>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_reference_trajectory

#include <boost/test/unit_test.hpp>
BOOST_AUTO_TEST_SUITE (reftraj_test) 
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
     ReferenceTrajectory rf(pts,refSpeed); 
     Eigen::Vector2d p;
     double val = 0;

     BOOST_CHECK_EQUAL( rf.points_.size(), 5);
     BOOST_CHECK_EQUAL( rf.segmentDistances_.size(), 4);
     BOOST_CHECK_EQUAL( rf.segmentUnitDirVectors_.size(), 4);
     BOOST_CHECK_EQUAL( rf.segmentUnitNormalVectors_.size(), 4);
     BOOST_CHECK_EQUAL( rf.segmentVectors_.size(), 4);
     BOOST_CHECK_EQUAL( rf.speeds_.size(), 4);
     
     BOOST_TEST_MESSAGE("Testing if vectors are properly initialized");
     for( int k = 0 ; k < rf.segmentUnitDirVectors_.size() ; k++ )
     {
     BOOST_CHECK_EQUAL( rf.segmentUnitDirVectors_[k].norm() , 1);
     BOOST_CHECK_EQUAL( rf.segmentUnitNormalVectors_[k].norm() , 1);
     BOOST_CHECK_EQUAL( rf.segmentUnitNormalVectors_[k].dot(rf.segmentUnitDirVectors_[k]) , 0);
     }
     
}
BOOST_AUTO_TEST_CASE( test_getProjectionOnSegment )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,0));
     pts.push_back(Eigen::Vector2d(2.5,0));
     pts.push_back(Eigen::Vector2d(3,0));
     pts.push_back(Eigen::Vector2d(4,0));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     ReferenceTrajectory rf(pts,refSpeed); 

     BOOST_TEST_MESSAGE("Testing if a point is properly projected on to a line segment");
     
     Eigen::Vector2d p;
     double perpDist = 0;
     double alongDist = 0;
     double parameter = 0;
     
     BOOST_TEST_MESSAGE("Testing if a point is properly projected in the middle of a line segment");
     p[0] = 0.5;
     p[1] = 0.0;
     rf.getProjectionOnSegment(perpDist,alongDist,parameter,p,0);
     BOOST_CHECK_EQUAL(perpDist, 0.0);
     BOOST_CHECK_EQUAL(alongDist, 0.5);
     BOOST_CHECK_EQUAL(parameter, 0.5);
     
     BOOST_TEST_MESSAGE("Testing if a point is properly projected on the right end point of line segment");
     p[0] = 1.5;
     p[1] = 0.0;
     rf.getProjectionOnSegment(perpDist,alongDist,parameter,p,0);
     BOOST_CHECK_EQUAL(perpDist, 0.5);
     BOOST_CHECK_EQUAL(alongDist, 1.0);
     BOOST_CHECK_EQUAL(parameter, 1.0);
     
     BOOST_TEST_MESSAGE("Testing if a point is properly projected on the left end point of line segment");
     p[0] = -1.5;
     p[1] = 0.0;
     rf.getProjectionOnSegment(perpDist,alongDist,parameter,p,0);
     BOOST_CHECK_EQUAL(perpDist, 1.5);
     BOOST_CHECK_EQUAL(alongDist, 0.0);
     BOOST_CHECK_EQUAL(parameter, 0.0);
     
     BOOST_TEST_MESSAGE("Testing if a point is properly projected on the line segment of a certain length");
     p[0] = 1.5;
     p[1] = 0.0;
     rf.getProjectionOnSegment(perpDist,alongDist,parameter,p,1);
     BOOST_CHECK_EQUAL(perpDist, 0.0);
     BOOST_CHECK_EQUAL(alongDist, 0.5);
     BOOST_CHECK_EQUAL(parameter, 0.5/1.5);
}

BOOST_AUTO_TEST_CASE( test_getSegmentWindowFromScanWindow )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,0));
     pts.push_back(Eigen::Vector2d(2.5,0));
     pts.push_back(Eigen::Vector2d(3,0));
     pts.push_back(Eigen::Vector2d(4,0));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     ReferenceTrajectory rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 0;
     
     BOOST_TEST_MESSAGE("Testing if a full scan window is correctly converted to segment window ");
     rf.getSegmentWindowFromScanWindow(minSegmentIndex,maxSegmentIndex,0,4);
     BOOST_CHECK_EQUAL ( minSegmentIndex,0);
     BOOST_CHECK_EQUAL ( maxSegmentIndex,3);
     
     BOOST_TEST_MESSAGE("Testing if a middle scan window is correctly converted to segment window ");
     rf.getSegmentWindowFromScanWindow(minSegmentIndex,maxSegmentIndex,1.2,2.8);
     BOOST_CHECK_EQUAL ( minSegmentIndex,1);
     BOOST_CHECK_EQUAL ( maxSegmentIndex,2);
     
     BOOST_TEST_MESSAGE("Testing if under (negative start) scan window is correctly converted to segment window ");
     rf.getSegmentWindowFromScanWindow(minSegmentIndex,maxSegmentIndex,-1.2,2.8);
     BOOST_CHECK_EQUAL ( minSegmentIndex,0);
     BOOST_CHECK_EQUAL ( maxSegmentIndex,2);
     
     BOOST_TEST_MESSAGE("Testing if over scan window is correctly converted to segment window ");
     rf.getSegmentWindowFromScanWindow(minSegmentIndex,maxSegmentIndex,3,5);
     BOOST_CHECK_EQUAL ( minSegmentIndex,3);
     BOOST_CHECK_EQUAL ( maxSegmentIndex,3);
}

BOOST_AUTO_TEST_CASE( test_getDistanceOnLastSegment )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,0));
     pts.push_back(Eigen::Vector2d(2.5,0));
     pts.push_back(Eigen::Vector2d(3,0));
     pts.push_back(Eigen::Vector2d(4,0));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     ReferenceTrajectory rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 3;
     double cumDist = 0;
     double distFromPath = 0; // Need to verify
     Eigen::Vector2d point;
     Eigen::Vector2d refVel;
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (start) ");
     point[0] = 0.2;
     point[1] = 0.2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK_EQUAL(cumDist,0.2);
     BOOST_CHECK_EQUAL(distFromPath,0.2);
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = 1.5;
     point[1] = 0.5;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK_EQUAL(cumDist,1.5);
     BOOST_CHECK_EQUAL(distFromPath,0.5);
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (before end) ");
     point[0] = 3.0;
     point[1] = 5.2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK_EQUAL(cumDist,3.0);
     BOOST_CHECK_EQUAL(distFromPath,5.2);
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (end) ");
     point[0] = 3.2;
     point[1] = 2.2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK_EQUAL( cumDist , 3.2 );
     BOOST_CHECK_EQUAL(distFromPath,2.2);
     
     BOOST_TEST_MESSAGE("Testing if distance traveled is given (point outside range )");
     point[0] = 4.2;
     point[1] = 1.2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK_EQUAL( cumDist , 4 );
     BOOST_CHECK_EQUAL(distFromPath, std::sqrt(1.2*1.2 + 0.2*0.2) );
     
     BOOST_TEST_MESSAGE("Testing if distance traveled is given (point outside range )");
     point[0] = 2.8;
     point[1] = 0.2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK_EQUAL( cumDist , 2.8 );
     BOOST_CHECK_EQUAL(distFromPath,0.2);
}

BOOST_AUTO_TEST_CASE( test_getCumulativeDist_diag )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,1));
     pts.push_back(Eigen::Vector2d(2,2));
     pts.push_back(Eigen::Vector2d(3,3));
     pts.push_back(Eigen::Vector2d(4,4));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     ReferenceTrajectory rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 3;
     double cumDist = 0;
     double distFromPath = 0; // Need to verify
     Eigen::Vector2d point;
     Eigen::Vector2d refVel;
     
     double eps = 0.01;
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (start) ");
     point[0] = 0;
     point[1] = 1;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist - std::sqrt(2)/2 ) < eps );
     BOOST_CHECK( std::abs(distFromPath - std::sqrt(2)/2 ) < eps );
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = 0;
     point[1] = 2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist - std::sqrt(2) ) < eps );
     BOOST_CHECK( std::abs(distFromPath - std::sqrt(2) ) < eps );
}

BOOST_AUTO_TEST_CASE( test_getCumulativeDist_harder )
{
     std::vector<Eigen::Vector2d> pts;
     
     pts.push_back(Eigen::Vector2d(0,0));
     pts.push_back(Eigen::Vector2d(1,1));
     pts.push_back(Eigen::Vector2d(2,2));
     pts.push_back(Eigen::Vector2d(3,2));
     pts.push_back(Eigen::Vector2d(4,2));
     
     std::vector<double> refSpeed;
     refSpeed.assign(4,1);
     ReferenceTrajectory rf(pts,refSpeed); 

     int minSegmentIndex = 0;
     int maxSegmentIndex = 3;
     double cumDist = 0;
     double distFromPath = 0; // Need to verify
     Eigen::Vector2d point;
     Eigen::Vector2d refVel;

     double eps = 0.01;
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (start) ");
     point[0] = 0;
     point[1] = 1;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist - std::sqrt(2)/2 ) < eps );
     BOOST_CHECK( std::abs(distFromPath - std::sqrt(2)/2 ) < eps );
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = 0;
     point[1] = 2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist - std::sqrt(2) ) < eps );
     BOOST_CHECK( std::abs(distFromPath - std::sqrt(2) ) < eps );
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = -1.0;
     point[1] = 0;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist ) < eps );
     BOOST_CHECK( std::abs(distFromPath - 1 ) < eps );
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = 5.0;
     point[1] = 2;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist - (2 * std::sqrt(2) + 2)) < eps );
     BOOST_CHECK( std::abs(distFromPath - 1 ) < eps );
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = 6.0;
     point[1] = 0;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist - (2 * std::sqrt(2) + 2)) < eps );
     BOOST_CHECK( std::abs(distFromPath - 2*std::sqrt(2) ) < eps );
     
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = -1.0;
     point[1] = 1;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist) < eps );
     BOOST_CHECK( std::abs(distFromPath - std::sqrt(2) ) < eps );
     
     BOOST_TEST_MESSAGE("Testing if distance traveled on last segment is given (mid) ");
     point[0] = -1.0;
     point[1] = -1;
     rf.getCumulativeDistanceOnPath(cumDist, distFromPath, refVel,point,minSegmentIndex,maxSegmentIndex);
     BOOST_CHECK( std::abs(cumDist) < eps );
     BOOST_CHECK( std::abs(distFromPath - std::sqrt(2) ) < eps );
}
BOOST_AUTO_TEST_SUITE_END( )
