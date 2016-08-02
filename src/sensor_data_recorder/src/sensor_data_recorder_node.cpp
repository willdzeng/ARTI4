#include <sensor_data_recorder/sensor_data_recorder.h>
#include <ros/ros.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "sensor_data_recorder");

	tf::TransformListener tf_(ros::Duration(10));
	ros::NodeHandle nh;
	machine_learning::SensorDataRecorder sensor_data_recorder(tf_, nh);
	ros::spin();
	return(0);
}