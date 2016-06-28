#include <dad_local_planner/dad_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include "dad_local_planner/dad_planner.h"


namespace dad_local_planner
{
void DADPlannerROS::reconfigureCB ( DADPlannerROSConfig& ROSconfig, uint32_t level )
{
  // if ( setup_ && ROSconfig.restore_defaults )
  //     {
  //     ROSconfig = defaultConfig_;
  //     ROSconfig.restore_defaults = false;
  //     }
  printf ( "DADP_ROS : dynamic reconfiguration calling back\n" );
  if ( ! setup_ )
    {
      defaultConfig_ = ROSconfig;
      setup_ = true;
    }

  // update generic local planner params
  config_.spatialConvergenceThreshold= ROSconfig.spatial_convergence_threshold;
  config_.angularConvergenceThreshold= ROSconfig.angular_convergence_threshold;

  config_.robotRadius = ROSconfig.robot_radius;
  config_.trajResolution = ROSconfig.traj_resolution;
  config_.detourCostFactor = ROSconfig.detour_cost_factor;
  config_.tubeSize = ROSconfig.tube_size;
  // config_.mpPath = ROSconfig.mp_path;
  config_.maxIterationCount = ROSconfig.max_iteration_count;
  config_.globalFrameId = ROSconfig.global_frame_id;
  config_.mpDuration = ROSconfig.mp_duration;
  config_.debug = ROSconfig.debug;
  config_.debugWaitInput = ROSconfig.debug_wait_input;
  config_.spatialHashSize = ROSconfig.spatial_hash_size;
  config_.spatialHashTimeTolerance = ROSconfig.spatial_hash_time_tolerance;
  config_.accMaxFwd = ROSconfig.acc_max_fwd;
  config_.accMaxBack = ROSconfig.acc_max_back;
  config_.velMaxFwd = ROSconfig.vel_max_fwd;
  config_.velMaxBack = ROSconfig.vel_max_back;
  config_.curvatureMax = ROSconfig.curvature_max;
  config_.skipCostFactor = ROSconfig.skip_cost_factor;
  config_.staticObstacleCollisionRiskFactor = ROSconfig.static_obs_collision_risk_factor;
  config_.dynamicObstacleCollisionRiskFactor = ROSconfig.dynamic_obs_collision_risk_factor;

  config_.subOptimalityEps = ROSconfig.sub_optimality_eps;
  // config_.velMax = ROSconfig.vel_max;

  // update dwa specific configuration
  dp_->reconfigure ( config_ );
}



DADPlannerROS::DADPlannerROS() : initialized_ ( false ), setup_ ( false )
{
}

void DADPlannerROS::initialize (
  std::string name )
{
  if ( ! isInitialized() )
    {

      ros::NodeHandle privateNh ( "~/" + name );
      ros::NodeHandle n;

      ros::NodeHandle pnh ( "~" );

      pnh.param ( "dynamic_obstacle_topic", dynamicObstacleTopic_, std::string ( "/dynamic_obstacles" ) );
      pnh.param ( "reference_path_topic", referencePathTopic_, std::string ( "/ref_path" ) );
      pnh.param ( "mp_path", mpPath_, std::string ( "/motion_primitive.txt" ) );
      pnh.param ( "odom_topic", odomTopic_, std::string ( "/odometry/filtered" ) );

      // pnh.param ("robot_radius",config_.robotRadius, 1.0);
      // pnh.param ("traj_resolution",config_.trajResolution, 10);
      // pnh.param ("detour_cost_factor",config_.detourCostFactor, 2.0);
      // pnh.param ("tube_size",config_.tubeSize, 0.2);
      // pnh.param ("max_iteration_count",config_.maxIterationCount, 5000);
      // pnh.param ("global_frame_id",config_.globalFrameId,std::string("odom"));
      // pnh.param ("mp_duration",config_.mpDuration, 1.0);
      // ROS_INFO("DADPlannerROS: get mp duration %f",config_.mpDuration);

      // dp_->reconfigure ( config_ );

      ROS_INFO ( "DADPlannerROS: get mpPath is %s", mpPath_.c_str() );
      ROS_INFO ( "DADPlannerROS: get dynamic_obstacle_topic is %s", dynamicObstacleTopic_.c_str() );
      ROS_INFO ( "DADPlannerROS: get reference_path_topic is %s", referencePathTopic_.c_str() );

      globalPlanSub_ = n.subscribe ( referencePathTopic_, 1, &DADPlannerROS::globalPlanCallback, this );
      dynamicObstacleSub_ = n.subscribe ( dynamicObstacleTopic_, 1, &DADPlannerROS::dynamicObstaclesCallback, this );
      odometrySub_ = n.subscribe ( odomTopic_, 1, &DADPlannerROS::odometryCallback, this );

      localPlanPathPub_ = pnh.advertise<nav_msgs::Path> ( "local_plan", 1 );
      localPlanTrajPub_ = n.advertise<ugv_msgs::LocalPlan> ( "local_plan_trajectory", 1 );

      // make sure to update the costmap we'll use for this cycle

      tf_ = std::shared_ptr<tf::TransformListener> ( new tf::TransformListener ( ros::Duration ( 10 ) ) );
      staticCostMap_ = std::shared_ptr<costmap_2d::Costmap2DROS> ( new costmap_2d::Costmap2DROS ( "static_cost_map", *tf_ ) );


      // ROS_INFO ( "DADP_ROS : initialized costmaps" );
      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = std::shared_ptr<DADPlanner> ( new DADPlanner ( name , staticCostMap_, mpPath_ ) );
      dynamicObstaclesVectorPtr_ =  std::shared_ptr<std::vector<dynamic_obstacle::DynamicObstacle> > ( new std::vector<dynamic_obstacle::DynamicObstacle>() );
      // ROS_INFO ( "DADP_ROS : initialized actual planner" );

      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<DADPlannerROSConfig> ( pnh );
      dynamic_reconfigure::Server<DADPlannerROSConfig>::CallbackType cb = boost::bind ( &DADPlannerROS::reconfigureCB, this, _1, _2 );
      dsrv_->setCallback ( cb );

    }
  else
    {
      ROS_WARN ( "This planner has already been initialized, doing nothing." );
    }
}

void DADPlannerROS::globalPlanCallback ( const nav_msgs::Path::ConstPtr& msg )
{
  ROS_INFO ( "Received global plan. Attempting to dispatch it to planner" );
  setReferenceTrajectory ( msg->poses );
}

void DADPlannerROS::dynamicObstaclesCallback ( const ugv_msgs::DynamicObstacles::ConstPtr& obstacles )
{
  // if (got_obstacle_ == false){
  dynamicObstaclesVectorPtr_->clear();
  ROS_INFO_ONCE ( "DADPlannerROS got obstacle" );
  double x, y, vx, vy, radius, sigma;
  for ( int i = 0; i < obstacles->obstacles.size(); i++ )
    {
      std::vector<double> pose;
      std::vector<double> vel;
      pose.push_back ( obstacles->obstacles[i].pose.position.x );
      pose.push_back ( obstacles->obstacles[i].pose.position.y );
      pose.push_back ( obstacles->obstacles[i].pose.position.z );
      vel.push_back ( obstacles->obstacles[i].twist.linear.x );
      vel.push_back ( obstacles->obstacles[i].twist.linear.y );
      vel.push_back ( obstacles->obstacles[i].twist.linear.z );
      radius = obstacles->obstacles[i].radius;
      sigma = obstacles->obstacles[i].sigma;
      dynamic_obstacle::DynamicObstacle ob ( pose, vel, radius, sigma );
      dynamicObstaclesVectorPtr_->push_back ( ob );
    }
  // got_obstacle_ = true;
  // }

  //
}

void DADPlannerROS::odometryCallback ( const nav_msgs::Odometry::ConstPtr& odometryMsg )
{
  geometry_msgs::Pose p = odometryMsg->pose.pose;
  tf::Quaternion q ( p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w );
  tf::Matrix3x3 m ( q );
  double roll, pitch, yaw;
  m.getRPY ( roll, pitch, yaw );
  
  currentPositionX_ = p.position.x;
  currentPositionY_ = p.position.y;
  currentHeading_ = yaw;
}


void DADPlannerROS::plan()
{
  //ROS_INFO(" DADP_ROS : In plan ");

  bool searchFinished = dp_->plan();

  if ( searchFinished == true )
    {
      ros::Duration searchDuration = ros::Time::now() - startTime_;
      ROS_INFO ( "Plan search finished , time used is %f\n",searchDuration.toSec() );
      publishLocalPlan();
    }
  //ROS_INFO(" DADP_ROS : Out plan ");
}

bool DADPlannerROS::setReferenceTrajectory ( const std::vector<geometry_msgs::PoseStamped>& orig_global_plan )
{
  if ( ! isInitialized() )
    {
      ROS_ERROR ( "This planner has not been initialized, please call initialize() before using this planner" );
      return false;
    }

  ROS_INFO ( "DADP_ROS : Setting global plan on actual planner" );
  dp_->setReferenceTrajectory ( orig_global_plan );
  dp_->setDynamicObstacles ( dynamicObstaclesVectorPtr_ );
  dp_->startSearch(currentPositionX_,currentPositionY_,currentHeading_);
  startTime_ = ros::Time::now();
  /// need to find a way to receive obstacles and triggle dp_.search().
}

void DADPlannerROS::publishLocalPlan()
{
  nav_msgs::Path path;
  ugv_msgs::LocalPlan localPlan;
  std::string frameId = "odom";
  path.header.frame_id = frameId.c_str();
  std::vector<Eigen::Vector3d> poseV;
  std::vector<double> timeV;
  dp_->getSolution ( poseV, timeV );
  double x, y, th, t;

  for ( int i = 0 ; i < poseV.size(); i++ )
    {
      x = poseV[i][0];
      y = poseV[i][1];
      th = poseV[i][2];
      t = timeV[i];
      // get path msgs
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 0;
      poseStamped.pose.orientation.z = th;
      poseStamped.pose.orientation.w = 1;
      poseStamped.header.frame_id = frameId.c_str();
      poseStamped.pose.position.x = x;
      poseStamped.pose.position.y = y;
      path.poses.push_back ( poseStamped );
      // get trajectory msgs
      geometry_msgs::Pose pose;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = th;
      pose.orientation.w = 1;
      pose.position.x = x;
      pose.position.y = y;
      localPlan.poses.push_back ( pose );
      localPlan.time.push_back ( t );
    }
  localPlanPathPub_.publish ( path );
  localPlanTrajPub_.publish ( localPlan );
}

DADPlannerROS::~DADPlannerROS()
{
  delete dsrv_;
}



};
