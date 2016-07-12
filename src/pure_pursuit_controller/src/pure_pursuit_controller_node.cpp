#include <ros/ros.h>

#include <pure_pursuit_controller/pure_pursuit_controller.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit_controller");
  ros::NodeHandle nh("~");
  try {
    PurePursuitController node(nh);
    node.spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }
  return 0;
}
