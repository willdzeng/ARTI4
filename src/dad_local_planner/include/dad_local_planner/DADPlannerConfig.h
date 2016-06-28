#pragma once
#include <string>
#include <vector>

namespace dad_local_planner{
	class DADPlannerConfig{
	public:
		DADPlannerConfig(){

		}
		~DADPlannerConfig(){}
		
		double spatialConvergenceThreshold;
		double angularConvergenceThreshold;

		double robotRadius;
		int trajResolution;
		double detourCostFactor;
		double tubeSize;
		int maxIterationCount;
		double mpDuration;
		std::string globalFrameId;
		std::string mpPath;
		bool debug;
		bool debugWaitInput;
		int spatialHashSize;
		double spatialHashTimeTolerance;
		double accMaxFwd;
		double accMaxBack;
		// double velMax;
		double velMaxFwd;
		double velMaxBack;
		double curvatureMax;
		double skipCostFactor;
		double staticObstacleCollisionRiskFactor;
		double dynamicObstacleCollisionRiskFactor;
		
		double subOptimalityEps;
	};
}