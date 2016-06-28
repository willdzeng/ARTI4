#pragma once

namespace dad_local_planner{
	class MotionPrimitiveInfo{

	public:
		MotionPrimitiveInfo(){
			vmin = -0.5;
			vmax = 1;
			vc = 0;
			valid = false;
			// index = -1;
		}
		~MotionPrimitiveInfo(){}

		double vmin;
		double vmax;
		double vc;
		bool valid;
		// double index;
	};
}