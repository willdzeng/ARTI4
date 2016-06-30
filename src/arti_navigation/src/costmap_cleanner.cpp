// #include <string>
// #include <vector>
// #include <iostream>
// #include <vector>
// #include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <math.h>

// namespace arti_navigation {
// 	class CostmapCleanner{
// 		CostmapCleanner(){

// 		}

// 		~CostmapCleanner(){

// 		}
// 	};
// }


int main(int argc, char **argv)
{
	ros::init(argc, argv, "costmap_cleanner");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string service_name;
	double frequency;
	pn.param("service_name", service_name, std::string("clear_costmaps"));
	pn.param("frequency", frequency, 1.0);
	ROS_INFO("Got service_name: %s", service_name.c_str());
	ROS_INFO("Got frequency %f", frequency);
	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(service_name);
	ros::Rate r(frequency);
	std_srvs::Empty srv;
	while (n.ok()) {
		try {
			client.call(srv);
			ROS_INFO("Called clear_costmaps service_name");
		}catch(std::runtime_error& x){

		}
		r.sleep();
	}
	return 0;
}