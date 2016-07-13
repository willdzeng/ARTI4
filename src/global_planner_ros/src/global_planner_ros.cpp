
#include <global_planner_ros/global_planner_ros.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

namespace global_planner_ros {

  GlobalPlannerROS::GlobalPlannerROS(tf::TransformListener& tf) :
    tf_(tf),
    planner_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    planner_plan_(NULL),
    collision_check_frequency_(10)
    {

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    //get some parameters that will be global to the move base node
    std::string global_planner_ros;
    private_nh.param("base_global_planner", global_planner_ros, std::string("navfn/NavfnROS"));

    if( !private_nh.getParam( "robot_namespace", robot_namespace_ )){
      robot_namespace_ = "";
    }

    if( !private_nh.getParam( "global_frame", global_frame_ )){
      global_frame_ = "/odom";
    }

    if( !private_nh.getParam( "robot_frame", robot_frame_ )){
      robot_frame_ = "/base_link";
    }

    // if (strcmp( const(global_frame_.c_str()[0]),"/")) {
    //   global_frame_ = "/"+global_frame_;
    // }
    
    // global_frame_ = robot_namespace_ + global_frame_;
    // robot_frame_ = robot_namespace_ + robot_frame_;
    ROS_INFO("robot namespace is %s",robot_namespace_.c_str());
    
    // ROS_INFO("robot_frame is %s",robot_frame_.c_str());

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    new_planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan",1);

    goal_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&GlobalPlannerROS::planningCallBack, this, _1));

    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    

    //initialize the global planner
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!bgp_loader_.isClassAvailable(global_planner_ros)){
        std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(global_planner_ros == bgp_loader_.getName(classes[i])){
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                global_planner_ros.c_str(), classes[i].c_str());
            global_planner_ros = classes[i];
            break;
          }
        }
      }

      planner_ = bgp_loader_.createInstance(global_planner_ros);
      planner_->initialize(bgp_loader_.getName(global_planner_ros), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner_ros.c_str(), ex.what());
      exit(1);
    }

    planner_costmap_ros_->start();
    global_frame_ = planner_costmap_ros_->getGlobalFrameID();
    robot_frame_ = planner_costmap_ros_->getBaseFrameID();
    ROS_INFO("goblal_frame is %s",global_frame_.c_str());
    ROS_INFO("robot_frame is %s",robot_frame_.c_str());

    collision_check_thread_ = new boost::thread(boost::bind(&GlobalPlannerROS::planCollisionCheck, this)); 
    //advertise a service for getting a plan
    //make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlannerROS::planService, this);
    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_global_costmaps", &GlobalPlannerROS::clearCostmapsService, this);
    //we're all set up now so we can start the action server
    dsrv_ = new dynamic_reconfigure::Server<global_planner_ros::GlobalPlannerROSConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<global_planner_ros::GlobalPlannerROSConfig>::CallbackType cb = boost::bind(&GlobalPlannerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void GlobalPlannerROS::reconfigureCB(global_planner_ros::GlobalPlannerROSConfig &config, uint32_t level)
  {
    //The first time we're called, we just want to make sure we have the
    //original configuration
    collision_check_frequency_ = config.collision_check_frequency;
    smooth_tolerance_ = config.smooth_tolerance;
    smooth_delta_ = config.smooth_delta;
    smooth_weight_ = config.smooth_weight;
  }

  void GlobalPlannerROS::planningCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal){
    global_goal_ptr_ = goal;
    if(!isQuaternionValid(goal->pose.orientation)){
      ROS_WARN("Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    global_goal_ = goalToGlobalFrame(*goal);
    geometry_msgs::PoseStamped temp_goal = global_goal_;
    printf("%s, global planner get goal, %f  %f  %f\n",ros::this_node::getName().c_str(),
      temp_goal.pose.position.x, temp_goal.pose.position.y, temp_goal.pose.position.z);

    planner_plan_->clear();
    ros::NodeHandle n;
    bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

    if(gotPlan){
      // smoothPath(*planner_plan_, *new_planner_plan_);
      new_planner_plan_ = planner_plan_;
      new_plan_ = true;
      base_local_planner::publishPlan(*new_planner_plan_, plan_pub_);
      printf("%s published a plan which has size %lu\n",ros::this_node::getName().c_str(), new_planner_plan_->size());
    }else{
      ROS_WARN("Global Planner has something wrong with Planning Check if the goal has the right format");
    }
  }

  // void GlobalPlannerROS::replan(const geometry_msgs::PoseStamped goal){
  //   if(!isQuaternionValid(goal.pose.orientation)){
  //     ROS_WARN("Aborting on goal because it was sent with an invalid quaternion");
  //     return;
  //   }
  //   //global_goal_ = goalToGlobalFrame(goal);
  //   geometry_msgs::PoseStamped temp_goal = goal;
  //   printf("%s global planner replanning get goal, %f  %f  %f\n",ros::this_node::getName().c_str(),
  //     temp_goal.pose.position.x, temp_goal.pose.position.y, temp_goal.pose.position.z);

  //   planner_plan_->clear();
  //   ros::NodeHandle n;
  //   bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
  //   if(gotPlan){
  //     // smoothPath(*planner_plan_, *new_planner_plan_);
  //     new_planner_plan_ = planner_plan_;
  //     new_plan_ = true;
  //     base_local_planner::publishPlan(*new_planner_plan_, plan_pub_);
  //   }else{
  //     ROS_WARN("Global Planner has something wrong with Planning Check if the goal has the right format");
  //   }
  // }

  bool GlobalPlannerROS::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    planner_costmap_ros_->resetLayers();
    return true;
  }

  GlobalPlannerROS::~GlobalPlannerROS(){
    delete dsrv_;
    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;
    delete planner_plan_;
    planner_.reset();
    // delete bgp_loader_;
  }

  bool GlobalPlannerROS::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    // tf::Stamped<tf::Pose> global_pose;
    // if(!planner_costmap_ros_->getRobotPose(global_pose)) {
    //   
    //   return false;
    // }

    geometry_msgs::PoseStamped start;
    if( ! getRobotPose(start)){
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }
    printf("robot start pose is %f %f %f\n",start.pose.position.x, start.pose.position.y, start.pose.orientation.z);
    // start.header.frame_id = global_frame_;
    // tf::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("global_planner_ros","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  bool GlobalPlannerROS::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped GlobalPlannerROS::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }

  void GlobalPlannerROS::planCollisionCheck(){
    ROS_INFO("setup global plan collision check thread");
    ros::NodeHandle n;
    costmap_2d::Costmap2D* costmap = planner_costmap_ros_->getCostmap();
    geometry_msgs::PoseStamped pose;
    unsigned int cell_x, cell_y;
    double x,y;
    double cost;
    bool collision;

    while(n.ok()){
      //printf("111\n");
      collision = false;
      //printf("middle plan collision_check\n");
      
      if (new_planner_plan_->size() > 0){
        //printf("2222\n");
        new_plan_ = false;
        int i = 0;
        while ( i < new_planner_plan_->size() && !new_plan_ && n.ok()){  //for (int i = 0; i < new_planner_plan_->size(); i++){
          pose = new_planner_plan_->at(i);
          x = pose.pose.position.x;
          y = pose.pose.position.y;      
          if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
            continue;
          }
          cost = double(costmap->getCost(cell_x, cell_y));
          if ((cost >= 254 && cost!=255)|| cost < 0){
            collision = true;
            break;
          }
          i++;
        }
      }

      if (collision){
        ROS_WARN("detected collision in current plan Replanning!!!");
        planningCallBack(global_goal_ptr_);
      }
      //printf("3333\n");
      ros::Rate r(collision_check_frequency_);
      r.sleep();

    }
  }

  void GlobalPlannerROS::smoothPath(std::vector<geometry_msgs::PoseStamped> old_path, std::vector<geometry_msgs::PoseStamped>& new_path) {
    new_path = old_path;
    double lDelta = smooth_tolerance_;

    while(lDelta>= smooth_tolerance_)
    {
      lDelta=0;
      for(int j = 1; j < old_path.size()-1; j++)
      {
        geometry_msgs::PoseStamped vertex_old = new_path[j];

        new_path[j].pose.position.x = new_path[j].pose.position.x + smooth_delta_ * ( old_path[j].pose.position.x - new_path[j].pose.position.x)+
          smooth_weight_ * ( new_path[j+1].pose.position.x + new_path[j-1].pose.position.x - 2 * new_path[j].pose.position.x);
        new_path[j].pose.position.y = new_path[j].pose.position.y + smooth_delta_ * (old_path[j].pose.position.y - new_path[j].pose.position.y)+
          smooth_weight_ * (new_path[j+1].pose.position.y + new_path[j-1].pose.position.y-2 * new_path[j].pose.position.y);

        lDelta = lDelta + hypot(vertex_old.pose.position.x - new_path[j].pose.position.x , vertex_old.pose.position.y - new_path[j].pose.position.y );
      }
    }
  }

  bool GlobalPlannerROS::getRobotPose(geometry_msgs::PoseStamped& pose){
    // last_pose.clear();
    double Pcx, Pcy, Pcth;
    ros::Time tTime;
    std::string error;
    tf::StampedTransform transform;
    try
    {
      ros::Time now = ros::Time::now();
      tf_.waitForTransform(global_frame_,
                           robot_frame_,
                           ros::Time(),
                           ros::Duration(1.0));
      tf_.lookupTransform(global_frame_, 
                          robot_frame_,
                          ros::Time(),
                          transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("tf failure: %s", ex.what());
      return false;
    }

    pose.pose.position.x = transform.getOrigin().getX();
    pose.pose.position.y = transform.getOrigin().getY();
    pose.pose.position.z = transform.getOrigin().getZ();
    pose.pose.orientation.z = tf::getYaw(transform.getRotation());
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = global_frame_;
    return true;
  }


  //   bool GlobalPlannerROS::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
  //   //make sure we have a costmap for our planner
  //   if(planner_costmap_ros_ == NULL){
  //     ROS_ERROR("global_planner_ros cannot make a plan for you because it doesn't have a costmap");
  //     return false;
  //   }
  //   tf::Stamped<tf::Pose> global_pose;
  //   if(!planner_costmap_ros_->getRobotPose(global_pose)){
  //     ROS_ERROR("global_planner_ros cannot make a plan for you because it could not get the start pose of the robot");
  //     return false;
  //   }
  //   geometry_msgs::PoseStamped start;
  //   //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
  //   if(req.start.header.frame_id == "")
  //     tf::poseStampedTFToMsg(global_pose, start);
  //   else
  //     start = req.start;

  //   //update the copy of the costmap the planner uses
  //   clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

  //   //first try to make a plan to the exact desired goal
  //   std::vector<geometry_msgs::PoseStamped> global_plan;
  //   if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
  //     ROS_DEBUG_NAMED("global_planner_ros","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
  //         req.goal.pose.position.x, req.goal.pose.position.y);

  //     //search outwards for a feasible goal within the specified tolerance
  //     geometry_msgs::PoseStamped p;
  //     p = req.goal;
  //     bool found_legal = false;
  //     float resolution = planner_costmap_ros_->getCostmap()->getResolution();
  //     float search_increment = resolution*3.0;
  //     if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
  //     for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
  //       for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
  //         for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

  //           //don't search again inside the current outer layer
  //           if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

  //           //search to both sides of the desired goal
  //           for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

  //             //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
  //             if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

  //             for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
  //               if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

  //               p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
  //               p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

  //               if(planner_->makePlan(start, p, global_plan)){
  //                 if(!global_plan.empty()){

  //                   //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
  //                   //(the reachable goal should have been added by the global planner)
  //                   global_plan.push_back(req.goal);

  //                   found_legal = true;
  //                   ROS_DEBUG_NAMED("global_planner_ros", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
  //                   break;
  //                 }
  //               }
  //               else{
  //                 ROS_DEBUG_NAMED("global_planner_ros","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
  //               }
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }

  //   //copy the plan into a message to send out
  //   resp.plan.poses.resize(global_plan.size());
  //   for(unsigned int i = 0; i < global_plan.size(); ++i){
  //     resp.plan.poses[i] = global_plan[i];
  //   }

  //   return true;
  // }


};
