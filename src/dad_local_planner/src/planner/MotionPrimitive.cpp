#include <dad_local_planner/MotionPrimitive.h>

using namespace dad_local_planner;  

void MotionPrimitive::append(const double nx, const double ny, const double nth, const double ns, const double nk) {
    x.push_back(nx);
    y.push_back(ny);
    th.push_back(nth);
    s.push_back(ns);
    k.push_back(nk);
}