
#pragma once
#include <dad_local_planner/DynamicsConstraints.h>
#include <dad_local_planner/MotionPrimitive.h>
#include <dad_local_planner/MotionPrimitiveInfo.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <cassert>
#include <algorithm>
#include <cmath>
// #include <boost/shared_ptr.hpp>
#include <dad_local_planner/TrajectorySpace.h>


namespace dad_local_planner
{

class MotionPrimitiveManager
{
private:
    std::shared_ptr< std::vector<MotionPrimitive> > mpDatabasePtr_;

    std::shared_ptr<DynamicsConstraints> dynamicsConstraints_;

    double DURATION_ONE_MP;
    double MAX_DISTANCE_ONE_MP;

    double mpSize_;

    void setMaxDistanceOneMP(); //> This is private since max distance can be computed based on MP database

    std::vector<MotionPrimitiveInfo> mpInfoVector_;

    TrajectorySpace trajSpace_;

public:
    MotionPrimitiveManager() {};
    MotionPrimitiveManager(const std::string fileName);

    void setDurationOneMP(const double durationOneMP);

    double getMaxDistanceOneMP() const;
    double getDurationOneMP() const;

    void setDynamicsConstraints(std::shared_ptr<DynamicsConstraints> dynamicsConstraints);

    void getFeasibleMotionPrimitiveIndexSet(std::vector<int>& mpIndexSet, const double currentVelocity, const double currentOmega);

    const MotionPrimitive& getMotionPrimitive(const int index) const;

    bool loadMotionPrimitives(const std::string fileName);

    const MotionPrimitiveInfo getMotionPrimitiveInfo(const int index) const;

    void setCurvatureConstraint(const double kmin, const double kmax);

    void setVelocityConstraint(const double vmin, const double vmax);

    void setTrajectorySpaceResolution(const int resolution);

    void reinitialize();

};

};