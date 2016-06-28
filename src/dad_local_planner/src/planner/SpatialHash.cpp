#include <dad_local_planner/SpatialHash.h>

using namespace dad_local_planner;  
/*
SpatialHash::SpatialHash():
SIZE_X(3),
SIZE_Y(3),
SIZE_TH(3),
MID_X(1),
MID_Y(1),
MID_TH(1),
MIN_X(-1),
MAX_X(1),
MIN_Y(-1),
MAX_Y(1),
MIN_TH(-1),
MAX_TH(1)
{
ps_.resize(boost::extents[SIZE_X][SIZE_Y][SIZE_TH]);
}
*/
SpatialHash::SpatialHash(double originX, double originY, int binsX, int binsY, int binsTH, Range rangeX, Range rangeY, Range rangeTH):
originX_(originX),
originY_(originY),
SIZE_X(binsX),
SIZE_Y(binsY),
SIZE_TH(binsTH),
MID_X(binsX/2),
MID_Y(binsY/2),
MID_TH(binsTH/2),
MIN_X(rangeX.first),
MAX_X(rangeX.second),
MIN_Y(rangeY.first),
MAX_Y(rangeY.second),
MIN_TH(rangeTH.first),
MAX_TH(rangeTH.second)
{
  
ps_.resize(boost::extents[SIZE_X][SIZE_Y][SIZE_TH]);
//Only want odd numbers as size to allow symmetric ranges
#ifdef ASSERTS
assert(SIZE_X % 2 == 1);
assert(SIZE_Y % 2 == 1);
assert(SIZE_TH % 2 == 1);
#endif
}

PoseSpaceIndex SpatialHash::poseToIndex(double x, double y, double th) const
{
    double offsetX = x - originX_;
    double offsetY = y - originY_;
    /*
  std::cout << " Input x : " << x << " , y : " << y << std::endl;
  std::cout << " Origin x : " << originX_ << " , y : " << originY_ << std::endl;
  std::cout << " Offset x : " << offsetX << " , y : " << offsetY << std::endl;
  */
  
    PoseSpaceIndexType binX = (offsetX / (MAX_X-MIN_X)) * SIZE_X + MID_X;
    PoseSpaceIndexType binY = (offsetY / (MAX_Y-MIN_Y)) * SIZE_Y + MID_Y;
    PoseSpaceIndexType binTH = (th / (MAX_TH-MIN_TH)) * SIZE_TH + MID_TH;
/*
    std::cout << std::endl; 
    std::cout << " For : " << x << " " << y << " " << th << std::endl; 
    std::cout << " Giving out : " << binX << " " << binY << " " << binTH << std::endl; 
    std::cout << std::endl; 
    */
#ifdef ASSERTS
    assert( binX >= 0 && binX < SIZE_X);
    assert( binY >= 0 && binY < SIZE_Y);
    assert( binTH >=0 && binTH < SIZE_TH);
#endif


    return boost::make_tuple(binX,binY,binTH);
}

void SpatialHash::insertVertex(double x,double y, double th, NodeIdType nodeId)
{
    PoseSpaceIndex ind = poseToIndex(x,y,th);
    insertVertex_(boost::get<0>(ind),boost::get<1>(ind),boost::get<2>(ind), nodeId);
}

void SpatialHash::insertVertex_(PoseSpaceIndexType binX,PoseSpaceIndexType binY, PoseSpaceIndexType binTH, NodeIdType nodeId)
{
    ps_[binX][binY][binTH].push_back(nodeId);
}

int SpatialHash::getVertexCountAt(PoseSpaceIndex ind) const
{
    int binX = boost::get<0>(ind);
    int binY = boost::get<1>(ind);
    int binTH = boost::get<2>(ind);
    
    return getVertexCountAt_(binX,binY,binTH);
}

int SpatialHash::getVertexCountAt_(PoseSpaceIndexType binX,PoseSpaceIndexType binY,PoseSpaceIndexType binTH) const 
{
#ifdef ASSERTS
    assert( binX >= 0 && binX < SIZE_X);
    assert( binY >= 0 && binY < SIZE_Y);
    assert( binTH >=0 && binTH < SIZE_TH);
#endif
    return ps_[binX][binY][binTH].size();
}

const std::vector<NodeIdType>& SpatialHash::getNodeIdVectorAt(double x, double y, double th) const
{
  PoseSpaceIndex ind = poseToIndex(x,y,th);
  return ps_[boost::get<0>(ind)][boost::get<1>(ind)][boost::get<2>(ind)];
}

void SpatialHash::dump()
{
  std::ofstream f;
  std::cout << " Dumping " << std::endl;
  f.open("/home/rpradeep/Desktop/dump.txt");
  for(int i = 0;i < SIZE_X; i++)
  {
    for(int j = 0;j< SIZE_Y; j++)
    {
      int m = 0;
      for (int k = 0;k< SIZE_TH; k++)
      {
	m += getVertexCountAt_(i,j,k);
      }
      f << m << " ";
    }
    f << "\n";
  }
  f.close();
}



