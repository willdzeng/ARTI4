
#include <global_planner_ros/global_planner_ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "global_planner_node");
  tf::TransformListener tf(ros::Duration(10));

  global_planner_ros::GlobalPlannerROS global_planner_ros( tf );

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}
