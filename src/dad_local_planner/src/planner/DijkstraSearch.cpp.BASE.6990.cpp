#include <dad_local_planner/DijkstraSearch.h>

using namespace dad_local_planner;

/**
 * Test doc
 */

DijkstraSearch::DijkstraSearch():
    robotRadius_(1), trajResolution_(10), tubeSize_(0.2), detourCostFactor_(2.0),debug_(false),debugWaitInput_(false),spatialHashSize_(201)
{
    //std::cout << " Motion primitives are not loaded by default " << std::endl;
}
/**
 * Initializes (or reinitializes) the DijkstraSearch object with fresh data structures by clearing all data structures.
 * This is crucial for replanning using the same DijkstraSearch object.
 * @param[in] costMapPtr A pointer to the cost map used in the collision collisionCostEvaluator_
 * @param[in] refTrajPtr A pointer to the reference trajectory given by higher level planner
 * @param[in] startNode The initial node for the search
 */
void DijkstraSearch::initialize(std::shared_ptr<costmap_2d::Costmap2DROS> costMapPtr,
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

    tubeCostEvaluator_.initialize ( refTrajPtr_, lookAheadDistance_ , detourCostFactor_ , tubeSize_ ); // look ahead is 1.0 ( this has to scale with the motion primitives used ) SEARCH_PERF
    staticObstacleCostEvaluator_.initialize ( costMapPtr_ );

    dynamicObstacleCostEvaluator_.initialize(*dynamicObstaclesVector);
    dynamicObstacleCostEvaluator_.setRobotRadius(robotRadius_);
    dynamicObstacleCostEvaluator_.setTrajStepNum(trajResolution_);
    // dynamicObstacleCostEvaluator_.setCostrains();

    // Setup the starting node
    startNode_ = startNode;


    // Setup the goal location
    Eigen::Vector2d lastPoint = refTrajPtr_->getLastPoint();
    goalNode_.x = lastPoint[0];
    goalNode_.y = lastPoint[1];
    // TODO ANGLE !!!

    // Clear data from previous runs
    solution_.clear();
    nodes_.clear();
    priorityQueue_ = std::priority_queue<Node, std::vector<Node>, NodeComparator>();

    // Gets the constrained mp index set for this planner run
    mpIndexSet_.clear();
    mpManagerPtr_->getFeasibleMotionPrimitiveIndexSet(mpIndexSet_, -1, -1);


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

void DijkstraSearch::reconfigure(const DADPlannerConfig& config) {
    // config_ = config;
    // doce_.setRobotRadius(config.robotRadius);
    // doce_.setTrajStepNum(config.trajectoryResolution);
    robotRadius_ = config.robotRadius;
    trajResolution_ = config.trajResolution;
    detourCostFactor_ = config.detourCostFactor;
    tubeSize_ = config.tubeSize;
    debug_ = config.debug;
    debugWaitInput_ = config.debugWaitInput;
    spatialHashSize_ = config.spatialHashSize;

    accMaxFwd_ = config.accMaxFwd;
    accMaxBack_ = config.accMaxBack;
    velMax_ = config.velMax;

    printf("DijkstraSearch : dynamic reconfiguration: Setting robotRadius_ to %f \n", robotRadius_);
    printf("DijkstraSearch : dynamic reconfiguration: Setting trajResolution_ to %d \n", trajResolution_);
    printf("DijkstraSearch : dynamic reconfiguration: Setting tubeSize_ to %f \n", tubeSize_);
    printf("DijkstraSearch : dynamic reconfiguration: Setting detourCostFactor_ to %f \n", detourCostFactor_);
    // look ahead is 1.0 ( this has to scale with the motion primitives used ) SEARCH_PERF
    lookAheadDistance_ = 2 * mpManagerPtr_->getMaxDistanceOneMP();
    tubeCostEvaluator_.initialize ( refTrajPtr_, lookAheadDistance_ , detourCostFactor_ , tubeSize_ );
    dynamicObstacleCostEvaluator_.setRobotRadius(robotRadius_);
    dynamicObstacleCostEvaluator_.setTrajStepNum(trajResolution_);
    dynamicObstacleCostEvaluator_.setConstraints(accMaxFwd_,accMaxBack_,velMax_);
    // printf("DijkstraSearch : dynamic reconfiguration : mp duration is %f\n",mpManagerPtr_->getDurationOneMP());
}
bool DijkstraSearch::search(int maxIterationCount)
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
        // printf("DijkstraSearch: search started maximum iteration count is %d\n", maxIterationCount);
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
bool DijkstraSearch::canNodeBeSkipped ( const Node& currentNode ) const
{

    //std::cout << " DijkstraSearch canNodeBeSkipped " << std::endl;

    bool skipThisNode = false;
    const std::vector<NodeIdType>& vec = spatialHashPtr_->getNodeIdVectorAt ( currentNode.x, currentNode.y, currentNode.th );
    for ( std::vector<NodeIdType>::const_iterator it = vec.begin(); it != vec.end() ; it++ )
    {

        NodeIdType id = *it;
        const Node existingNode = nodes_[id];
        //if (  existingNode.gCost < currentNode.gCost ) // here cost is same as node.gCost .. this is before adding heuristics #### MINEFIELD
        if ( std::abs(existingNode.t - currentNode.t)<0.05 && existingNode.gCost < currentNode.gCost ) // here cost is same as node.gCost .. this is before adding heuristics #### MINEFIELD
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
void DijkstraSearch::pushNodeIntoQueue(Node &childNode)
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
void DijkstraSearch::populateChildNodeCosts(Node &childNode, const Node &parentNode, const MotionPrimitive &transformedMp)
{
    dynamicObstacleCostEvaluator_.populateTransitionData(childNode, parentNode , transformedMp); // dynamic obstalce cost;
    staticObstacleCostEvaluator_.populateTransitionData ( childNode, parentNode , transformedMp ); // this sets the childNode cCost
    tubeCostEvaluator_.populateTransitionData ( childNode, parentNode , transformedMp ); // this sets the childNode state, path pos and tCost

    childNode.hCost = 0;
    childNode.gCost = parentNode.gCost + childNode.tCost + childNode.cCost + childNode.dCost;
    childNode.totalCost = childNode.gCost + childNode.hCost;
}

/**
 * Neighboring nodes to the currentNode are added in this function. More precisely, currentNode is expanded and the motion primitives from the motion primitive
 * set are added to the queue.
 * @param[in] currentNode The node to expand.
 */
void DijkstraSearch::addNeighbors ( const Node& currentNode )
{
    for ( int k = 0 ; k < mpIndexSet_.size() ; k++ )
    {
        // Obtain a reference to the motion primitive in the database
        const MotionPrimitive mp = mpManagerPtr_->getMotionPrimitive(mpIndexSet_[k]);

        Node childNode;
        childNode.parentId = currentNode.id;

        // Transform function defined in node.cpp takes care of childNode x,y,th,t,mpid and also gives a transformed motion primitive starting at the parentNode
        MotionPrimitive transformedMp(mp);
        transformMotionPrimitiveAndChildNode(transformedMp, childNode, currentNode);

        populateChildNodeCosts(childNode, currentNode, transformedMp);

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
bool DijkstraSearch::checkGoal ( const Node& node ) const
{
    // Need to make sure the node is near goal point and also has a cumulative distance close to the end point
    // CORRECTNESS need to be fixed with proper velocity info
    if ( std::abs ( node.x - goalNode_.x ) < 0.5 && std::abs ( node.y - goalNode_.y ) < 0.5)
    {
        std::cout << " ########### Near goal ############ " << std::endl;
	
	if ( node.pathPosition + 0.5 < refTrajPtr_->getLastCumulativeDistance() )
	{
	  std::cout << " ########### But, the path pos is not near enough ############ " << std::endl;
	  return false;
	}
        //std::cout << " Node path pos : " << node.pathPosition << " , Last segment cum dist : " << refTrajPtr_->getLastSegmentCumulativeDistance() << std::endl;

        // Stages cleared may used for accelerating the search

        
        if (  node.state == Node::STATE::FOLLOW )
        {
            // std::cout << " ########### Found goal ############ " << std::endl;
            //std::cout << node << std::endl;
            return true;
        }
        else
        {
            return false;
        }
        

    }
    else
    {
        return false;
    }
}
/**
 * This function produces the solution that will be published.
 * @param[in] node Node to be traced back to the root node in the tree.
 */
void DijkstraSearch::traceBack ( const Node& node )
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
void DijkstraSearch::setMotionPrimitiveManager(std::shared_ptr< MotionPrimitiveManager > motionPrimitiveManagerPtr)
{
    mpManagerPtr_ = motionPrimitiveManagerPtr;
}

// const std::vector< Eigen::Vector2d >& DijkstraSearch::getSolution() const
//     {
//       return solution_;
//     }
const std::vector<Node>& DijkstraSearch::getSolution() const
{
    return solution_;
}