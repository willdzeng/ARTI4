#pragma once

#include <dad_local_planner/MotionPrimitive.h>
#include <cassert>
#include <vector>
#include <cmath>
#include <iostream>

namespace dad_local_planner
{
class Trajectory
{
private:
  void interpolate ( const MotionPrimitive &motionPrimitive )
  {

    assert ( motionPrimitive.ds >= 0 && " Backward motion primitives are not supported at the moment " );

    const int mpSize = motionPrimitive.x.size() ;


    // Transition time where acceleration has to be stopped
    double tBar = std::abs( vf - vi ) / ( aMax );
    //assert(tBar <= duration && "Maximum acceleration constraint disallows the velocity jump.");

    // Transition s ; The s position when the acceleration has to be stopped.
    double sBar = vi * tBar + 0.5 * aMax * tBar * tBar ;

    // Need to find a mapping from s to t in order to assign a time to each data point
    for ( int i = 0 ; i < mpSize ; i++ )
    {

      double xi = motionPrimitive.x[i];
      double yi = motionPrimitive.y[i];
      double thi = motionPrimitive.th[i];
      double si = motionPrimitive.s[i];

      x.push_back(xi);
      y.push_back(yi);
      th.push_back(thi);

      double tt = 0;
      double vv = 0;
      if ( motionPrimitive.s[i] < sBar )
      {
        // we are still accelerating
        tt = ( -vi + std::sqrt(vi * vi + 2 * aMax * si ) ) / aMax;
        vv = vi + aMax * tt;
      }
      else
      {
        // we are done accelerating
        tt = tBar + ( motionPrimitive.s[i] - sBar ) / vf;
        vv = vf;
      }
      if(tt < -0.00001){
        printf("Trajectory: Time is negative, something wrong.\n");
        printf("            tt is %f, vi is %f, vf is %f, aMax is %f, s[i] is %f, sBar is %f, tBar is %f\n",
                            tt,vi,vf,aMax,si,sBar, tBar);
        assert(0);
      }

      tt = std::abs(tt); //  incase of small -0.00000 shows up
      if(tt > 10000){
        printf("Trajectory: Time is infinite, somthing seriously wrong\n");
        printf("            tt is %f, vi is %f, vf is %f, aMax is %f, s[i] is %f, sBar is %f, tBar is %f\n",
                            tt,vi,vf,aMax,si,sBar, tBar);
        assert(0);
      }

      
      
      t.push_back(ti + tt);
      v.push_back(vv);

      /*
               std::cout <<"\t" ;
               std::cout <<" s : " << motionPrimitive.s[i];
               std::cout <<" , x  : " << xi;
               std::cout <<" , y : " << yi;
               std::cout <<" , th : " << thi;
               std::cout <<" , t : " << tt + ti ;
               std::cout <<" , v : " << vv ;
               std::cout << std::endl;
         */

    }
    tf = t.back();
    duration = tf - ti;
    assert(duration >= 0 && " Duration is negative. Watch out ");


  }
public:
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> th;
  std::vector<double> t;
  std::vector<double> v;

  double vi;
  double vf;
  /*
   * The acceleration that was used to move from vi to vf. Note that this is the max acceleration. So, it is possible that vf is reached before the duration of this trajectory.
   * After vf is reached, there is no more acceleration. And, vf stays the same.
   */
  double aMax;

  double ti;
  double tf;

  double duration;

  Trajectory ( const MotionPrimitive & motionPrimitive, const double initialVelocity, const double finalVelocity, const double maximumFwdAcceleration, const double maximumRevAcceleration, const double initialTime)
  {

    vi = initialVelocity;
    vf = finalVelocity;

    if ( vf < vi )
    {
      aMax = maximumRevAcceleration;
    }
    else
    {
      aMax = maximumFwdAcceleration;
    }

    double sRequired = ( vf * vf - vi * vi ) / aMax / 2;

    if (motionPrimitive.ds < sRequired) {
      printf("Trajectory: MotionPrimitive ds is less than the required amount. Trying to achieve desired velocity as best as possible.\n");
      // assert(0);
    }

    ti = initialTime;
    tf = -100;
    duration = -100;




    interpolate ( motionPrimitive );
  }

};

}
