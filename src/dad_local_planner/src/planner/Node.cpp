#include <dad_local_planner/Node.h>

using namespace dad_local_planner;  

Node::Node()
{
    state = Node::STATE::FOLLOW;

    x = 0;
    y = 0;
    th = 0;
    t = 0;

    pathPosition = 0;


    pathPositionAtStartDeviation = 0;
    pathPositionAtEndDeviation = 0;

    avgDistFromPath = 0;

    gCost = 0;
    hCost = 0;
    tCost = 0;
    cCost = 0;
    dCost = 0;
    doCost = 0;
    
    hCostFake = 0;

    totalCost = 0;
    
    mpId = -1;
    parentId = -1;
    id = -1;
}
bool Node::operator==(const Node& n) {
    double dsq = (x - n.x) * (x - n.x) + (y - n.y) * (y - n.y);
    double thetaDist = std::abs(th - n.th);
    while (thetaDist > M_PI) {
	thetaDist = 2 * M_PI - thetaDist;
    }
    if (dsq < LINEAR_DIST_THRESH && thetaDist < ANGULAR_DIST_THRESH) {
	return true;
    } else {
	return false;
    }
}
double Node::getDistSqFrom(double x_, double y_) const
{
  return ( x_ - x ) * ( x_ - x ) + ( y_ - y) * ( y_ - y);
}

namespace dad_local_planner
{
  

std::ostream& operator<<(std::ostream& os, const Node& nd) {
    os << " Node data - " << "\n";
    if ( nd.state == Node::STATE::FOLLOW )
    {
    os << " state : " << "FOLLOW" << std::endl;
    }
    else if ( nd.state == Node::STATE::DETOUR )
    {
      os << " state : " << "DETOUR" << std::endl;
    }
    os << " x : " << nd.x << " , " << " y : " << nd.y << " , " << " th : " << nd.th << "\n" ;
    os << " t : " << nd.t << "\n" ;
    os << " v : " << nd.v << "\n" ;
    os << " tcost : " << nd.tCost << " , mpid : " << nd.mpId << " , " <<" gcost : " << nd.gCost << "\n";
    os << " ccost : " << nd.cCost << " ,  hcost : " << nd.hCost << "\n";
    os << " totalCost : " << nd.totalCost << "\n";
    os << " path pos : " << nd.pathPosition << " , " << " dCost : " << nd.dCost << "\n";
    os << " pathStartDev : " << nd.pathPositionAtStartDeviation << " , pathEndDev : " << nd.pathPositionAtEndDeviation << "\n";
    os << " avgDistFromPath : " << nd.avgDistFromPath << "\n";
    os << " parentId : " << nd.parentId << "\n";
    os << " id : " << nd.id << "\n";
    return os;
}
void normalizeAngle(double& angle)
{
    // Normalize angle
    if ( angle > M_PI )
    {
      angle -= 2*M_PI;
    }
    if ( angle < -M_PI )
    {
      angle += 2*M_PI;
    }
    
}
void transformMotionPrimitiveAndChildNode(MotionPrimitive& mp, Node& childNode, const Node& parentNode)
{
  
  //std::cout << " Now setting mpid : " << childNode.mpId <<" to node "<< childNode.id << std::endl;
  
  // Now work on transforming the motion primitive
  const int M = mp.x.size();
  for ( int k = 0; k < M ; k++ )
  {
    using namespace std;
    double tx = cos(parentNode.th) * mp.x[k] - sin(parentNode.th) * mp.y[k]; 
    double ty = sin(parentNode.th) * mp.x[k] + cos(parentNode.th) * mp.y[k]; 
    mp.x[k] = parentNode.x + tx;
    mp.y[k] = parentNode.y + ty;
    double angle = parentNode.th + mp.th[k];
    // Normalize angle
    normalizeAngle(angle);
    if ( angle > M_PI )
    {
      angle -= 2*M_PI;
    }
    if ( angle < -M_PI )
    {
      angle += 2*M_PI;
    }
    mp.th[k] = angle;
  }
  // First set the childNode pose
  childNode.x = mp.x[M-1];
  childNode.y = mp.y[M-1];
  childNode.th = mp.th[M-1]; // already normalized
  childNode.t = parentNode.t + 1;
  childNode.mpId = mp.mpId;
}
void transformMotionPrimitive(MotionPrimitive& mp, const Node& parentNode)
{
  
  //std::cout << " Now setting mpid : " << childNode.mpId <<" to node "<< childNode.id << std::endl;
  
  // Now work on transforming the motion primitive
  const int M = mp.x.size();
  for ( int k = 0; k < M ; k++ )
  {
    using namespace std;
    double tx = cos(parentNode.th) * mp.x[k] - sin(parentNode.th) * mp.y[k]; 
    double ty = sin(parentNode.th) * mp.x[k] + cos(parentNode.th) * mp.y[k]; 
    mp.x[k] = parentNode.x + tx;
    mp.y[k] = parentNode.y + ty;
    double angle = parentNode.th + mp.th[k];
    // Normalize angle
    if ( angle > M_PI )
    {
      angle -= 2*M_PI;
    }
    if ( angle < -M_PI )
    {
      angle += 2*M_PI;
    }
    mp.th[k] = angle;
  }
}

}