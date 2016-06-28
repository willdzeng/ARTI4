#include <dad_local_planner/SpatialHash.h>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_spatial_hash

#include <boost/test/unit_test.hpp>

using namespace dad_local_planner;  

BOOST_AUTO_TEST_SUITE (spatialHash_test) 
BOOST_AUTO_TEST_CASE( test_init )
{
  SpatialHash sp(0,0,11,11,11,Range(-5,5),Range(-5,5),Range(-7,7));
  BOOST_TEST_MESSAGE("Testing if spatial hash is properly initialized");
  
}
BOOST_AUTO_TEST_SUITE_END( )
