#include <dad_local_planner/DynamicObstacleCostEvaluator.h>


namespace dad_local_planner
{


DynamicObstacleCostEvaluator::DynamicObstacleCostEvaluator() :
  accMaxFwd_ ( 5.0 ), accMaxBack_ ( 5.0 ), robotRadius_ ( 1.0 ), collisionRiskFactor_(1.0)
{

}
DynamicObstacleCostEvaluator::~DynamicObstacleCostEvaluator()
{

}

void DynamicObstacleCostEvaluator::initialize ( const std::vector<dynamic_obstacle::DynamicObstacle>& dynamicObstaclesVector )
{
  dynamicObstaclesVector_ = dynamicObstaclesVector;
  printf ( "DOCE got a dynamic_obstacle vector size is %ld\n", dynamicObstaclesVector_.size() );
}
/*
void DynamicObstacleCostEvaluator::updateObstacles ( const std::vector<dynamic_obstacle::DynamicObstacle>& dynamicObstaclesVector )
{
  dynamicObstaclesVector_ = dynamicObstaclesVector;
}
*/

void DynamicObstacleCostEvaluator::setRobotRadius ( const double robotRadius )
{
  robotRadius_ = robotRadius;
  printf ( "DOCE: Setting robot radius %f\n", robotRadius_ );
}

void DynamicObstacleCostEvaluator::setTrajStepNum ( const int trajStepNum )
{
  trajStepNum_ = trajStepNum;
  printf ( "DOCE: Setting trajectory step number %d\n", trajStepNum_ );
}

void DynamicObstacleCostEvaluator::setCollisionRiskFactor( const double riskFactor)
{
  collisionRiskFactor_ = riskFactor;
}
void DynamicObstacleCostEvaluator::setAccConstraints ( const double accMaxFwd, const double accMaxBack )
{
  accMaxFwd_ = accMaxFwd;
  accMaxBack_ = accMaxBack;
  printf ( "DOCE: Setting Constraints:  accMaxFow:  %f\n", accMaxFwd_ );
  printf ( "DOCE: Setting Constraints:  accMaxBack: %f\n", accMaxBack_ );
}

void DynamicObstacleCostEvaluator::moveAllObstacles ( const std::vector<double>& timeVector )
{

  timeVector_ = timeVector;

  if ( timeVector_.size() == 0 )
    {
      assert ( 0 && " DynamicObstacle : No time set up can't move obstacle" );
      return;
    }

  dynamic_obstacle::DynamicObstacle* ob;
  double xt, yt, t;
  for ( int i = 0 ; i < dynamicObstaclesVector_.size(); i++ )
    {
      ob = &dynamicObstaclesVector_[i];
      ob->clear();
      //printf("get obstalce pose %f %f, velocity %f %f\n",ob->x, ob->y, ob->vx, ob->vy);
      for ( int j = 0; j < timeVector.size(); j++ )
        {
          xt = ob->x + ob->vx * timeVector[j];
          yt = ob->y + ob->vy * timeVector[j];
          ob->x_pts.push_back ( xt );
          ob->y_pts.push_back ( yt );
          // printf("obstacle pose at time %f, x: %f y: %f\n",timeVector[j],xt,yt);
        }
    }
}


void DynamicObstacleCostEvaluator::populateTransitionData ( Node& childNode, const Node& parentNode, const MotionPrimitive& mp )
{

  if ( dynamicObstaclesVector_.size() == 0 )
    {
      // assert( 0 && " DOCE :  No obstacles given");
      childNode.doCost = 0;
      // childNode.v = traj.vf;
      return;
    }

  // double finalVelocity = velMax_; //temperally setting up velocity as maximum veloicty;

  const Trajectory traj ( mp, parentNode.v, childNode.v, accMaxFwd_, accMaxBack_, parentNode.t );

  moveAllObstacles ( traj.t );
  // printf("move_obstacle success\n");

  double cost = 0;
  double dist = 0;
  dynamic_obstacle::DynamicObstacle * ob;
  for ( int j = 0; j < traj.x.size(); j++ )
    {
      for ( int i = 0; i < dynamicObstaclesVector_.size(); i++ )
        {

          ob = &dynamicObstaclesVector_[i];
          // printf("get obstalce pose %f %f, velocity %f %f sigma %f \n",ob->x, ob->y, ob->vx, ob->vy, ob->sigma);
          dist = distFunc ( traj.x[j], traj.y[j], ob->x_pts[j], ob->y_pts[j] );
          // printf("dist is %f \n",dist);
          dist = dist - robotRadius_ - ob->radius;
          // printf("dist is %f \n",dist);
          if ( dist <= 0 )
            {
              goto has_collision;
              // printf("collision cost is 100000\n");
            }
          cost += gaussian(dist,ob->sigma);
          // cost += 0;
          // printf("dist: %f cost: %f \n",dist,cost);
        }
    }
  cost /= dynamicObstaclesVector_.size() * traj.x.size();
  // printf("cost calculation success\n");
  childNode.doCost = collisionRiskFactor_ *  cost;
  childNode.v = traj.v.back();
  childNode.t = traj.tf;
  return;

has_collision:
  childNode.doCost = 1000; // collision
  childNode.v = traj.v.back();
  childNode.t = traj.tf;

}

double DynamicObstacleCostEvaluator::distFunc ( const double x1, const double y1, const double x2, const double y2 ) const
{
  return sqrt ( pow ( x2 - x1, 2 ) + pow ( y2 - y1, 2 ) );
}

double DynamicObstacleCostEvaluator::gaussian ( const double x, const double s ) const
{
  double a = x / s;
  return exp ( - a * a );
}
};
