#include <dad_local_planner/dad_planner.h>


namespace dad_local_planner
{

DADPlanner::STATUS DADPlanner::getPlannerStatus()
{
    return status_;
}

void DADPlanner::reconfigure(DADPlannerConfig& config)
{
    config_ = config;
    maxIterationCount_ = config.maxIterationCount;
    mpDuration_ = config.mpDuration;
    // get acceleration just for recover the plan.
    accMaxFwd_ = config.accMaxFwd; 
    accMaxBack_ = config.accMaxBack;
    velMaxFwd_ = config.velMaxFwd;
    velMaxBack_ = config.velMaxBack;
    curvatureMax_ = config.curvatureMax;

    // mpPath_ = config.mpPath;
    trajResolution_ = config.trajResolution;
    globalFrameId_ = config.globalFrameId;

    printf("DADP : dynamic reconfiguration: Setting globalFrameId_ to     %s\n", globalFrameId_.c_str());
    printf("DADP : dynamic reconfiguration: Setting maxIterationCount_ to %d\n", maxIterationCount_);
    printf("DADP : dynamic reconfiguration: Setting accMaxFow_ to         %f\n",accMaxFwd_);
    printf("DADP : dynamic reconfiguration: Setting accMaxBack_ to        %f\n",accMaxBack_);
    printf("DADP : dynamic reconfiguration: Setting velMaxFwd_ to         %f\n",velMaxFwd_);
    printf("DADP : dynamic reconfiguration: Setting velMaxBack_ to        %f\n",velMaxBack_);
    printf("DADP : dynamic reconfiguration: Setting curvatureMax_ to      %f\n",curvatureMax_);
    printf("DADP : dynamic reconfiguration: Setting trajResolution_ to    %d\n",trajResolution_);

    dynamicsConstraints_->maxForwardAcceleration = accMaxFwd_;
    dynamicsConstraints_->maxForwardVelocity = velMaxFwd_;
    dynamicsConstraints_->maxBackwardAcceleration = accMaxBack_;
    dynamicsConstraints_->maxBackwardVelocity = velMaxBack_;

    motionPrimitiveManagerPtr_->setDynamicsConstraints(dynamicsConstraints_);
    motionPrimitiveManagerPtr_->setDurationOneMP(mpDuration_);
    motionPrimitiveManagerPtr_->setCurvatureConstraint(-1*curvatureMax_,curvatureMax_);
    motionPrimitiveManagerPtr_->setVelocityConstraint(velMaxBack_,velMaxFwd_*2);
    motionPrimitiveManagerPtr_->setTrajectorySpaceResolution(trajResolution_);
    motionPrimitiveManagerPtr_->reinitialize();
    aSearch_.reconfigure(config);
    // aSearch_.setMotionPrimitiveManager(motionPrimitiveManagerPtr_);
    
}

DADPlanner::DADPlanner(std::string name , std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr, std::string mpPath):
    globalFrameId_(std::string("odom")), trajResolution_(200), mpDuration_(1.0)
{
    ros::NodeHandle private_nh("~/" + name);

// turn on this if dynamic reconfigration dosen't work
    // globalFrameId_ = std::string("odom");
    // trajResolution_ = 30;
    // mpDuration_ = 1;
    // mpPath_ = "/home/zeng/ugv_catkin_ws/src/other/matlab_scripts/trajgen/motion_primitives.txt";
    // maxIterationCount_ = 5000;
// ###################################################
    // mpPath_ = mpPath;

    costMapPtr_ = costMapPtr;
    printf("DADP : Setting cost map\n");
    printf("DADP : Setting globalFrameId_ to %s \n", globalFrameId_.c_str());
    printf("DADP : Get MP path %s \n", mpPath.c_str());

    status_ = DADPlanner::STATUS::INITIALIZED;
    // file_path << ros::package::getPath("dynamic_obstacle_publisher") << "/config/DO_initial_conditions.ini";

    dynamicsConstraints_ = std::shared_ptr<DynamicsConstraints>(new DynamicsConstraints());

    motionPrimitiveManagerPtr_ = std::shared_ptr<MotionPrimitiveManager>(new MotionPrimitiveManager( mpPath));



    motionPrimitiveManagerPtr_->setDurationOneMP(mpDuration_);
    aSearch_.setMotionPrimitiveManager(motionPrimitiveManagerPtr_);

}

bool DADPlanner::plan()
{
    //std::cout << " In DADPlanner plan " << std::endl;
    bool searchFinished = false;
    switch ( status_ )
    {
    case DADPlanner::STATUS::PLANNING:
        searchFinished = aSearch_.search(maxIterationCount_);
        if ( searchFinished == true )
        {
            std::cout << " Planner is done with search " << std::endl;
            status_ = DADPlanner::STATUS::READY;
            std::cout << " Planner is switching to ready status" << std::endl;
	    double elapsedTime = searchTimer_.elapsed();
            std::cout << " Planner took : " << elapsedTime * 1000 << " ms " << std::endl;
            std::cout << " Average time per expansion : " << (elapsedTime / aSearch_.expansions_) * 1000 << " ms" << std::endl;
            std::cout << " Average expansion per ms: " << aSearch_.expansions_ / elapsedTime / 1000 << std::endl;
        }
        break;
    }

    return searchFinished;
    //std::cout << " Out DADPlanner plan " << std::endl;
}

void DADPlanner::getSolution(std::vector<Eigen::Vector3d> &positionList, std::vector<double> &timeList) const
{
    std::vector<Node> nodeList = aSearch_.getSolution();
    if ( nodeList.size() <= 1 ) {
        assert( "node list has zero size, please check" && 0);
        return;
    }
    for (int i = nodeList.size() - 1 ; i > 0 ; i--)
    {
        // printf("000\n");
        Node dummyParent = nodeList[i];
        Node dummyChild = nodeList[i - 1];
        const MotionPrimitive mp = motionPrimitiveManagerPtr_->getMotionPrimitive(dummyChild.mpId);
        MotionPrimitive transformedMp = mp;
        // printf("111\n");
        transformMotionPrimitive( transformedMp, dummyParent );


        // for (int j = 0 ; j < transformedMp.x.size(); j++){
        // file << 0 << " " << transformedMp.x[j] << " " << transformedMp.y[j] << "\n";
        // }


        double v0 = dummyParent.v;
        double v1 = dummyChild.v;
        double currentTime = dummyParent.t;
        double duration = dummyChild.t - dummyParent.t;
        // double MAX_FWD_ACCEL = 2;
        // double MAX_REV_ACCEL = 2;

        const Trajectory traj(transformedMp, v0, v1, accMaxFwd_, accMaxBack_ , currentTime);
        for (int j = 0 ; j < traj.t.size(); j++) {
            positionList.push_back(Eigen::Vector3d(traj.x[j], traj.y[j], traj.th[j]) );
            timeList.push_back(traj.t[j]);
        }
    }

    return;
}

bool DADPlanner::setReferenceTrajectory(const std::vector<geometry_msgs::PoseStamped>& points)
{
    bool returnValue = false;
    if ( status_ == DADPlanner::STATUS::INITIALIZED || status_ == DADPlanner::READY ) {
        points_ = points;
        std::vector<Eigen::Vector2d> pts;

        for (int i = 0; i < points_.size(); i++) {
            geometry_msgs::PoseStamped pose = points_[i];
            pts.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
        }
        // reference speed has to be set by calculation but it is ok for now
        std::vector<double> refSpeed;
        refSpeed.assign(pts.size(), 1);

        refTraj_ = std::shared_ptr<WaypointPath> (new WaypointPath(pts, refSpeed));

        assert(pts.size() > 1 && "Are you sure you set the reference trajectory ? ");
        returnValue = true;
    }
    return returnValue;
}

void DADPlanner::setDynamicObstacles(std::shared_ptr< std::vector<dynamic_obstacle::DynamicObstacle> > dynamicObstaclesVectorPtr) {
    dynamicObstaclesVectorPtr_ = dynamicObstaclesVectorPtr;
}

void DADPlanner::startSearch(double x,double y,double th) {

    replan:
    if (dynamicObstaclesVectorPtr_ == NULL || refTraj_ == NULL || costMapPtr_ == NULL) {
        assert( 0 && "dad planner didn't initialize properly");
    }

    if ( status_ == DADPlanner::STATUS::INITIALIZED | status_ == DADPlanner::READY )
    {
        Node startNode;
        // get first two points and calculate angle;
	/*
    	Eigen::Vector2d point0 = refTraj_->getPoint(0);
    	Eigen::Vector2d point1 = refTraj_->getPoint(1);
        double x0 = point0[0];
        double y0 = point0[1];
        double x1 = point1[0];
        double y1 = point1[1];
        double th = std::atan2(y1 - y0, x1 - x0); //calculate angle.
        */
        startNode.x = x;
        startNode.y = y;
        startNode.th = th;
        startNode.t = 0;
        startNode.v = 0;
        startNode.id = 0;
        startNode.state = Node::STATE::FOLLOW;
        std::cout << " Starting search : " << std::endl;
        std::cout << " x : " << startNode.x << std::endl;
        std::cout << " y : " << startNode.y << std::endl;
        std::cout << " th : " << startNode.th << std::endl;

        aSearch_.initialize(costMapPtr_ , refTraj_, dynamicObstaclesVectorPtr_, startNode);
        status_ = DADPlanner::STATUS::PLANNING;
	searchTimer_.restart();
    }
    else if ( status_ == DADPlanner::STATUS::PLANNING ) {
        std::cout << " Interrupt received. Triggering replan." << std::endl;
	status_ = DADPlanner::STATUS::READY; 
	goto replan;
    }
}

};
