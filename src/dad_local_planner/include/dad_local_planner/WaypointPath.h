#pragma once

#include <iostream>
#include <cassert>
#include <iomanip>
#include <fstream>
#include <eigen3/Eigen/Dense>

namespace dad_local_planner
{
class WaypointPath
{
public:
    std::vector<Eigen::Vector2d> points_;
    std::vector<double> speeds_;
    std::vector<double> cumulativeDistances_;
    std::vector<double> segmentDistances_;
    std::vector<double> segmentDistancesSq_;
public:
    WaypointPath();
    WaypointPath(const std::vector<Eigen::Vector2d>& pointList, const std::vector<double>& desiredSpeedList);
    WaypointPath(const WaypointPath& waypointPath);

    void getWindowIndex(int& minIndex, int& maxIndex, const double minDist, const double maxDist) const;
    void getDistanceData(double& cumulativeDistance, double& distanceFromPath, Eigen::Vector2d& referenceVelocity, const Eigen::Vector2d& point, const int minPointId, const int maxPointId) const;

    const Eigen::Vector2d& getPoint(int index) const;
    const Eigen::Vector2d& getLastPoint() const;
    
    double getSpeed(int index) const;
    
    double getCumulativeDistance(int index) const;
    double getLastCumulativeDistance() const;

};

}