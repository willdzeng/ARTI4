#pragma once
#include <memory>
#include <vector>
#include <cmath>
#include <dynamic_obstacle/DynamicObstacle.h>
#include <assert.h>
#include <dad_local_planner/Trajectory.h>
#include <dad_local_planner/Node.h>
#include <boost/scoped_ptr.hpp>
#include <fstream>
#include <ostream>

namespace dad_local_planner{


	class DynamicObstacleCostEvaluator{
	
	public:
		DynamicObstacleCostEvaluator();

		~DynamicObstacleCostEvaluator();

		void initialize(const std::vector<dynamic_obstacle::DynamicObstacle>& dynamicObstaclesVector);

		/*
		void updateObstacles(const std::vector<dynamic_obstacle::DynamicObstacle>& dynamicObstaclesVector);
		*/

		void setRobotRadius(const double robotRadius);
		
		void setCollisionRiskFactor( const double riskFactor);

		void setTrajStepNum(const int trajStepNum);

		void moveAllObstacles(const std::vector<double>& timeVector);

		void populateTransitionData(Node& childNode,const Node& parentNode, const MotionPrimitive& mp);

		inline double distFunc(const double x1,const double y1,const double x2,const double y2) const;

		inline double gaussian(const double x, const double s) const;

		void setAccConstraints(const double accMaxFwd, const double accMaxBack);


	private:
		std::vector<dynamic_obstacle::DynamicObstacle> dynamicObstaclesVector_;
		double collisionRiskFactor_;
		double robotRadius_;
		int trajStepNum_;
		std::vector<double> timeVector_;
		double accMaxFwd_,accMaxBack_;
	};

	

}