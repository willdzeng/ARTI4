#pragma once

#include <iostream>
#include <ostream>
#include <cmath>
#include <boost/graph/graph_concepts.hpp>

#include "MotionPrimitive.h"

#define LINEAR_DIST_THRESH (0.5)
#define ANGULAR_DIST_THRESH (0.5)
namespace dad_local_planner
{
void normalizeAngle(double& angle);
class Node
{
public:
    enum STATE { FOLLOW, DETOUR };

    Node();
    bool operator==(const Node& n);
    double getDistSqFrom(double x_, double y_) const;
    friend std::ostream& operator<<(std::ostream& os, const Node& nd);
    friend void transformMotionPrimitiveAndChildNode(MotionPrimitive& mp, Node& childNode, const Node& parentNode);
    friend void transformMotionPrimitive(MotionPrimitive& mp, const Node& parentNode);
    
    STATE state;

    double x;
    double y;
    double th;
    double t;
    double v;

    double pathPosition;


    double pathPositionAtStartDeviation;
    double pathPositionAtEndDeviation;

    double avgDistFromPath;

    double dCost; //> Detour cost
    double cCost; //> Collision cost
    double gCost; //> Cost to come
    double hCost; //> Cost to go
    double tCost; //> Transition time cost
    double doCost; //> dynamic obstacle cost
    
    double hCostFake; //> used for tie breaking

    double totalCost;
    
    int mpId;
    int parentId;
    int id;

};

};