#include <dad_local_planner/AStarSearch.h>

using namespace dad_local_planner;

/**
 * Test doc
 */

AStarSearch::AStarSearch():
    robotRadius_(1),
    trajResolution_(10),
    tubeSize_(0.2),
    detourCostFactor_(2.0),
    debug_(false),
    debugWaitInput_(false),
    spatialHashSize_(101),
    spatialHashTimeTolerance_(0.05),
    velMaxFwd_(1),
    skipCostFactor_(0),
    staticObstacleCollisionRiskFactor_(0),
    subOptimalityEps_(0),
    spatialConvergenceThreshold_(0.5),
    angularConvergenceThreshold_(3.14)
{
    //std::cout << " Motion primitives are not loaded by default " << std::endl;
}
/**
 * Initializes (or reinitializes) the AStarSearch object with fresh data structures by clearing all data structures.
 * This is crucial for replanning using the same AStarSearch object.
 * @param[in] costMapPtr A pointer to the cost map used in the collision collisionCostEvaluator_
 * @param[in] refTrajPtr A pointer to the reference trajectory given by higher level planner
 * @param[in] startNode The initial node for the search
 */
void AStarSearch::initialize(std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr,
                                std::shared_ptr<WaypointPath> refTrajPtr,
                                std::shared_ptr< std::vector<dynamic_obstacle::DynamicObstacle> > dynamicObstaclesVector,
                                const Node& startNode)
{

    refTrajPtr_ = refTrajPtr;
    costMapPtr_ = costMapPtr;

    // enable this if dynamic reconfigure dosen't work
    // detourCostFactor_ = 2.0;
    // tubeSize_ = 0.2;
    // robotRadius_ = 1;
    // trajResolution_ = 10;
    // double spatialHashSize_ = 101; 
    //###############################################
    
    lookAheadDistance_ = 2 * mpManagerPtr_->getMaxDistanceOneMP();

    tubeCostEvaluator_.initialize ( refTrajPtr_, lookAheadDistance_ , detourCostFactor_ , tubeSize_, skipCostFactor_); // look ahead is 1.0 ( this has to scale with the motion primitives used ) SEARCH_PERF
    staticObstacleCostEvaluator_.initialize ( costMapPtr_ , staticObstacleCollisionRiskFactor_ );

    dynamicObstacleCostEvaluator_.initialize(*dynamicObstaclesVector);
    // dynamicObstacleCostEvaluator_.setRobotRadius(robotRadius_);
    // dynamicObstacleCostEvaluator_.setTrajStepNum(trajResolution_);
    // dynamicObstacleCostEvaluator_.setCostrains();

    // Setup the starting node
    startNode_ = startNode;


    // Setup the goal location
    Eigen::Vector2d lastPoint = refTrajPtr_->getLastPoint();
    goalNode_.x = lastPoint[0];
    goalNode_.y = lastPoint[1];
    goalNode_.th = 0.0; //normalizeAngle(0);

    // Clear data from previous runs
    solution_.clear();
    nodes_.clear();
    priorityQueue_ = std::priority_queue<Node, std::vector<Node>, NodeComparator>();

    //size of spatial hash has to be just as big as required ( i.e. only as big as the search horizon )
    // here assumption is that search horizon is 40m by 40m centered around the current startnode
    spatialHashPtr_ = std::shared_ptr<SpatialHash>(new SpatialHash(startNode_.x, startNode_.y, spatialHashSize_, spatialHashSize_, 41, Range ( -20, 20 ), Range ( -20, 20 ), Range ( -4, 4 ) )); // SEARCH_PERF
    //std::cout << " Starting at " << startNode_.x << " , " << startNode_.y << std::endl;

    searchStarted_ = false;
    searchFinished_ = false;
    //logFile_.open ( "dumpTree.txt", std::fstream::out );
    count_ = 0;
    expansions_ = 0;

}

void AStarSearch::reconfigure(const DADPlannerConfig& config) {
    // config_ = config;
    // doce_.setRobotRadius(config.robotRadius);
    // doce_.setTrajStepNum(config.trajectoryResolution);
    spatialConvergenceThreshold_ = config.spatialConvergenceThreshold;
    angularConvergenceThreshold_ = config.angularConvergenceThreshold;
    
    robotRadius_ = config.robotRadius;
    trajResolution_ = config.trajResolution;
    detourCostFactor_ = config.detourCostFactor;
    tubeSize_ = config.tubeSize;
    debug_ = config.debug;
    debugWaitInput_ = config.debugWaitInput;
    spatialHashSize_ = config.spatialHashSize;
    spatialHashTimeTolerance_ = config.spatialHashTimeTolerance;

    accMaxFwd_ = config.accMaxFwd;
    accMaxBack_ = config.accMaxBack;
    velMaxFwd_ = config.velMaxFwd;
    velMaxBack_ = config.velMaxBack;
    skipCostFactor_ = config.skipCostFactor;
    staticObstacleCollisionRiskFactor_ = config.staticObstacleCollisionRiskFactor;
    dynamicObstacleCollisionRiskFactor_ = config.dynamicObstacleCollisionRiskFactor;
    
    subOptimalityEps_ = config.subOptimalityEps;

    printf("AStarSearch : dynamic reconfiguration: Setting spatialConvergenceThreshold_ to      %f \n", spatialConvergenceThreshold_);
    printf("AStarSearch : dynamic reconfiguration: Setting angularConvergenceThreshold_ to      %f \n", angularConvergenceThreshold_);
    printf("AStarSearch : dynamic reconfiguration: Setting robotRadius_ to      %f \n", robotRadius_);
    printf("AStarSearch : dynamic reconfiguration: Setting trajResolution_ to   %d \n", trajResolution_);
    printf("AStarSearch : dynamic reconfiguration: Setting tubeSize_ to         %f \n", tubeSize_);
    printf("AStarSearch : dynamic reconfiguration: Setting spatialHashSize_ to  %d \n", spatialHashSize_);
    printf("AStarSearch : dynamic reconfiguration: Setting spatialHashTimeTolerance_ to  %f \n", spatialHashTimeTolerance_);
    printf("AStarSearch : dynamic reconfiguration: Setting detourCostFactor_ to %f \n", detourCostFactor_);
    printf("AStarSearch : dynamic reconfiguration: Setting skipCostFactor_ to   %f \n", skipCostFactor_);
    printf("AStarSearch : dynamic reconfiguration: Setting staticObstacleCollisionRiskFactor_ to  %f \n", staticObstacleCollisionRiskFactor_);
    printf("AStarSearch : dynamic reconfiguration: Setting dynamicObstacleCollisionRiskFactor_ to  %f \n", dynamicObstacleCollisionRiskFactor_);
    printf("AStarSearch : dynamic reconfiguration: Setting subOptimalityEps_ to %f \n", subOptimalityEps_);
    
    // look ahead is 1.0 ( this has to scale with the motion primitives used ) SEARCH_PERF
    lookAheadDistance_ = 1 * mpManagerPtr_->getMaxDistanceOneMP();
    tubeCostEvaluator_.initialize ( refTrajPtr_, lookAheadDistance_ , detourCostFactor_ , tubeSize_,skipCostFactor_ );
    
    staticObstacleCostEvaluator_.initialize ( costMapPtr_ , staticObstacleCollisionRiskFactor_ );
    
    dynamicObstacleCostEvaluator_.setRobotRadius(robotRadius_);
    dynamicObstacleCostEvaluator_.setTrajStepNum(trajResolution_);
    dynamicObstacleCostEvaluator_.setAccConstraints(accMaxFwd_,accMaxBack_);
    dynamicObstacleCostEvaluator_.setCollisionRiskFactor(dynamicObstacleCollisionRiskFactor_);
    // printf("AStarSearch : dynamic reconfiguration : mp duration is %f\n",mpManagerPtr_->getDurationOneMP());
}
bool AStarSearch::search(int maxIterationCount)
{
    if ( searchStarted_ == false )
    {
        pushNodeIntoQueue(startNode_);
        searchStarted_ = true;
        searchFinished_ = false;
        // std::cout << " **** Search started **** " << std::endl;
    }
    if (searchStarted_ == true)
    {
        // std::cout << " Continuing search ... " << std::endl;
        iterationCount_ = maxIterationCount; // For this run we allow this many iterations
        // printf("AStarSearch: search started maximum iteration count is %d\n", maxIterationCount);
        while ( priorityQueue_.empty() == false && iterationCount_ > 0 && searchFinished_ == false )
        {
            iterationCount_ = iterationCount_ - 1;

            Node topNode = priorityQueue_.top();
            priorityQueue_.pop();
            Node currentNode = nodes_[topNode.id]; // priority queue contains copy of the nodes so ... need to access nodes_ by id
            bestCostSoFar_ = currentNode.totalCost;

            // Now check if there another node in the spatialHash that is better than this node
            if ( canNodeBeSkipped ( currentNode ) == true )
            {
                continue;
            }
            // Insert it into the hash table
            spatialHashPtr_->insertVertex ( currentNode.x, currentNode.y, currentNode.th, currentNode.id );

            if ( checkGoal ( currentNode ) == true )
            {
                searchFinished_ = true;
                traceBack ( currentNode );
                break;
            }

            addNeighbors ( currentNode );
            expansions_++;

        }
    }
    return searchFinished_;
}
/**
 * This function checks if a node can be skipped. A node is a candidate for skipping if there is another node that comes to the
 * same x,y,th position and at the same time. In this function, it is assumed that if a node falls in a particular
 * spatial hash bin, then they are coincident. So, only a further time check is needed. A node is finally skipped if it is a candidate and
 * the current gcost of the node is higher than the one that is already in the bin.
 * @param[in] currentNode The node to be examined for the possibility of skipping
 */
bool AStarSearch::canNodeBeSkipped ( const Node& currentNode ) const
{

    //std::cout << " AStarSearch canNodeBeSkipped " << std::endl;

    bool skipThisNode = false;
    const std::vector<NodeIdType>& vec = spatialHashPtr_->getNodeIdVectorAt ( currentNode.x, currentNode.y, currentNode.th );
    for ( std::vector<NodeIdType>::const_iterator it = vec.begin(); it != vec.end() ; it++ )
    {

        NodeIdType id = *it;
        const Node existingNode = nodes_[id];
        //if (  existingNode.gCost < currentNode.gCost ) // here cost is same as node.gCost .. this is before adding heuristics #### MINEFIELD
        if ( std::abs(existingNode.t - currentNode.t) < spatialHashTimeTolerance_ && existingNode.gCost < currentNode.gCost ) // here cost is same as node.gCost .. this is before adding heuristics #### MINEFIELD
        {
            //std::cout << "\t Skipping node " << node.id << " with cost = " << node.gCost << " because another node " << id << " has better cost = " << existingNode.gCost << std::endl;
            // we skip this node
            skipThisNode = true;
            break;
        }
    }
    //skipThisNode = false; // this is debug statement to easily disable node skipping
    return skipThisNode;
}
/**
 * This function handles any nodes going into the priority queue. If there is any other place a push happens, it is a bug.
 * @param[in,out] childNode Child node to be given an id and put into the queue.
 * @param[in] totalCost The priority ( total cost ) associated with the child node that is used for inserting into the priority queue
 */
void AStarSearch::pushNodeIntoQueue(Node &childNode)
{
    childNode.id = nodes_.size();
    nodes_.push_back ( childNode );
    priorityQueue_.push(childNode);
}

/**
 * This function is populates child node data based on each of the cost evaluators.
 * Each cost evaluator fills a specific cost variable within child node.
 * @param[in,out] childNode
 * @param[in] parentNode
 * @param[in] transformedMp Transformed motion primitive that starts at parent node and ends at child node
 */
bool AStarSearch::populateChildNodeCosts(Node &childNode, const Node &parentNode, const MotionPrimitive &transformedMp)
{
    dynamicObstacleCostEvaluator_.populateTransitionData(childNode, parentNode , transformedMp); // dynamic obstalce cost;
    
    
    staticObstacleCostEvaluator_.populateTransitionData(childNode, parentNode , transformedMp);    // this sets the childNode cCost
    tubeCostEvaluator_.populateTransitionData(childNode, parentNode , transformedMp);    // this sets the childNode state, path pos and tCost

    double distToTimeFactor =  ( mpManagerPtr_->getDurationOneMP() / mpManagerPtr_->getMaxDistanceOneMP() );
    
    childNode.hCostFake = ( (detourCostFactor_ + 1)* childNode.avgDistFromPath + std::abs(refTrajPtr_->getLastCumulativeDistance() - childNode.pathPosition)) * distToTimeFactor;
    
    //double hCostSimple = detourCostFactor_ * std::abs( childNode.avgDistFromPath - tubeSize_ );
    double hCostSimple = detourCostFactor_ * ( childNode.avgDistFromPath - tubeSize_ );
    
    
    childNode.hCost = ( subOptimalityEps_ + 1 ) * hCostSimple;
    
    childNode.gCost = parentNode.gCost + childNode.tCost + childNode.cCost + childNode.doCost + childNode.dCost;

    childNode.totalCost = childNode.gCost + childNode.hCost;
    
    if ( childNode.doCost > 0 )
    {
      return false;
    }
    
    return true;
}

/**
 * Neighboring nodes to the currentNode are added in this function. More precisely, currentNode is expanded and the motion primitives from the motion primitive
 * set are added to the queue.
 * @param[in] currentNode The node to expand.
 */
void AStarSearch::addNeighbors ( const Node& currentNode )
{
    // Gets the constrained mp index set for this planner run
    double currentVelocity = currentNode.v;
    mpManagerPtr_->getFeasibleMotionPrimitiveIndexSet(mpIndexSet_, currentVelocity, -100000);

    for ( int k = 0 ; k < mpIndexSet_.size() ; k++ )
    {
        // Obtain a reference to the motion primitive in the database
        const MotionPrimitive mp = mpManagerPtr_->getMotionPrimitive(mpIndexSet_[k]);

        Node childNode;
        childNode.parentId = currentNode.id;

        // Transform function defined in node.cpp takes care of childNode x,y,th,t,mpid and also gives a transformed motion primitive starting at the parentNode
        MotionPrimitive transformedMp(mp);
        transformMotionPrimitiveAndChildNode(transformedMp, childNode, currentNode);

	////////////////////////////////////////////////////////////////////////////////
	// EXPERIMENTAL
	double preferredVel = velMaxFwd_;
        childNode.v = preferredVel; //temporarily setting as maximum fwd velocity;
	
	bool validVelocity = false;
        validVelocity = populateChildNodeCosts(childNode, currentNode, transformedMp);
        
	/*
	int maxRetries = 2;
	bool validVelocity = false;
	while(validVelocity == false && maxRetries-- > 0)
	{
        childNode.v = childNode.v * 0.5; //temporarily setting as maximum fwd velocity;
        validVelocity = populateChildNodeCosts(childNode, currentNode, transformedMp);
	}
	if ( validVelocity == false )
	{
	  //printf("\n Tried 2 times with different velocities. All failed.\n");
	  return;
	}
	*/
	
	////////////////////////////////////////////////////////////////////////////////
	
        // Here we check if this node can be skipped
        if ( canNodeBeSkipped ( childNode ) == true  )
        {
            return;
        }
        /*
        * MAKE SURE THE ORDER OF THE FOLLOWING LINES IS NOT CHANGED
        */
        pushNodeIntoQueue(childNode); // THIS LINE HAS TO COME BEFORE insertVertex because the child node id is only set in pushNodeIntoQueue
        spatialHashPtr_->insertVertex ( childNode.x, childNode.y, childNode.th, childNode.id );

        
        if(debug_ == true){
            // std::cout <<  std::endl;
            // std::cout <<  childNode << std::endl;
            // std::cout <<  std::endl;
            std::vector<Eigen::Vector3d> curvePoints;
            for(int i = 0 ; i < transformedMp.x.size(); i++)
            {
              curvePoints.push_back(Eigen::Vector3d(transformedMp.x[i],transformedMp.y[i],0));
            }

            // if ( std::abs( childNode.cCost - 1000 ) < 0.01 )
            if ( childNode.doCost > 1 )
            {
              rdt.publishPersistentCurve("debug1", curvePoints,0.02,1.0,0.0,0);
            }
            else
            {
              rdt.publishPersistentCurve("debug1", curvePoints,0.01,0.2,0.7,0.1);
            }
            if(debugWaitInput_==true){
	      printf("Waiting for key press: ");
                std::cin.get();
            }
        }
    }
}

/**
 * This is the function that establishes what it means to find a solution.
 * It uses the goalNode_ to determine the spatial closeness to the goal.
 * @param[in] node The node to be checked for convergence
 */
bool AStarSearch::checkGoal ( const Node& node ) const
{
    // Need to make sure the node is near goal point and also has a cumulative distance close to the end point
    // CORRECTNESS need to be fixed with proper velocity info
  
    // We have a node near the end of the path
    if ( node.pathPosition + 1.5 * spatialConvergenceThreshold_ < refTrajPtr_->getLastCumulativeDistance() )
    {
      return false;
    }
    if (  node.state != Node::STATE::FOLLOW )
    {
	return false;
    }
    
    // We have a follow node at this stage 
    
    
    
    if ( std::abs ( node.x - goalNode_.x ) > spatialConvergenceThreshold_ || std::abs ( node.y - goalNode_.y ) > spatialConvergenceThreshold_)
    {
        return false;
    }
    
    // At this point we are near goal. But, need to check angle for convergence.
    
    double angleDiff = std::abs(goalNode_.th - node.th);
    normalizeAngle(angleDiff);
    if (angleDiff > angularConvergenceThreshold_)
    {
	printf("\n Failed at angle convergence \n");
	
	return false;
    }
    
    return true;
}
/**
 * This function produces the solution that will be published.
 * @param[in] node Node to be traced back to the root node in the tree.
 */
void AStarSearch::traceBack ( const Node& node )
{

    Node s = node;
    while ( 1 )
    {
        solution_.push_back(s);
        std::cout << s << std::endl;

        s = nodes_[s.parentId];

        if ( s.id == 0 ) break;
    }

    s = nodes_[0];
    solution_.push_back(s);

    std::cout << " Expansions : " << expansions_ << std::endl;
    std::cout << " Total cost : " << node.gCost << std::endl;
    
    //spatialHashPtr_->dump();
}
/**
 * This loads motion primitives from the given file and puts them into the mpSet variable
 * @param[in] fileName The fully qualified file path to the motion primitive file
 */
void AStarSearch::setMotionPrimitiveManager(std::shared_ptr< MotionPrimitiveManager > motionPrimitiveManagerPtr)
{
    mpManagerPtr_ = motionPrimitiveManagerPtr;
}

// const std::vector< Eigen::Vector2d >& AStarSearch::getSolution() const
//     {
//       return solution_;
//     }
const std::vector<Node>& AStarSearch::getSolution() const
{
    return solution_;
}