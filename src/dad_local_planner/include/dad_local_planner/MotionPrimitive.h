#pragma once
#include <vector>
#include <ostream>
namespace dad_local_planner
{
  
class MotionPrimitive
{
public:
    MotionPrimitive(): dx(-1), dy(-1), dth(-1), ds(-1), kMax(-1), mpId(-1) {};
    MotionPrimitive(const double ddx, const double ddy, const double ddth, const double dds, const double kkMax, const int mmpid): dx(ddx), dy(ddy), dth(ddth), ds(dds), kMax(kkMax), mpId(mmpid) {};
    void append(const double nx, const double ny, const double nth, const double ns, const double nk);

    friend std::ostream& operator<<(std::ostream& os, const MotionPrimitive& mp) {
    	os << " mp id : " << mp.mpId << "\n";
    	os << " dx : " << mp.dx << "\n" ;
    	os << " dy : " << mp.dy << "\n" ;
    	os << " dth : " << mp.dth << "\n" ;
        os << " ds : " << mp.ds << "\n" ;
        os << " kMax : " << mp.kMax << "\n" ;
    	for (int p = 0; p < mp.x.size() ; p++)
    	    os << "[" << mp.x[p] << ' ' << mp.y[p] << ' ' << mp.th[p] << ' ' << mp.s[p] << ' ' << mp.k[p] <<"]\n";
    	return os;
    }
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> th;
    std::vector<double> s;
    std::vector<double> k;

    double dx;
    double dy;
    double dth;
    double ds;
    double kMax;

    int mpId;
};

};
