#include "ros/ros.h"
#include "dad_local_planner/dad_planner_ros.h"
#include "tf/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dad_planner_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);
 
  dad_local_planner::DADPlannerROS planner;
  
  planner.initialize("local_planner");
  
  planner.setupViz(n);
  while (ros::ok())
  {
    planner.plan();
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
