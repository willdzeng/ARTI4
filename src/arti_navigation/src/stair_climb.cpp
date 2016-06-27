#include <ros/ros.h>

namespace arti_navigation {
	class StairClimb{
		StairClimb(ros::NodeHandle nh, ros::NodeHandle private_nh){
			nh_ = nh;
			private_nh_ = private_nh;
			private_nh_.param("climb_time", climb_time_, 10);
			private_nh_.param("goal_index", goal_index_, 1);
			
		}

		~StairClimb(){

		}

		private:
			ros::NodeHandle nh_;
			ros::NodeHandle private_nh_;
			double climb_time_;
			int goal_index_;
	};
}
