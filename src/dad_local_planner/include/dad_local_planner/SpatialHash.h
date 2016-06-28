#pragma once
#include <boost/functional/hash.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/multi_array.hpp>
#include <vector>
#include <cassert>
#include <fstream>
#include <iostream>

#define ASSERTS
typedef int NodeIdType;
typedef boost::multi_array< std::vector<NodeIdType> , 3 > PoseSpace;
typedef PoseSpace::index PoseSpaceIndexType; 

typedef std::pair<int,int> Range;
typedef boost::tuple<PoseSpaceIndexType,PoseSpaceIndexType,PoseSpaceIndexType> PoseSpaceIndex;

namespace dad_local_planner
{


/**
 * This class describes a rectangular grid of bins. A node ids can be put into each of the bins.
 * This binning scheme allows fast access to nearby nodes for a given node.
 */
class SpatialHash 
{
  /*
    const int SIZE_X;
    const int SIZE_Y;
    const int SIZE_TH;

    const int MID_X;
    const int MID_Y;
    const int MID_TH;

    const int MAX_X;
    const int MIN_X;
    const int MAX_Y;
    const int MIN_Y;
    const int MAX_TH;
    const int MIN_TH;
    */
    
  /**
   * These define the extents of the grid hash table
   */
    int SIZE_X; ///> Max bins in X dimension
    int SIZE_Y; ///> Max bins in Y dimension
    int SIZE_TH; ///> Max bins in TH dimension

    int MID_X; ///> Mid bin in X dimension
    int MID_Y; ///> Mid bin in Y dimension
    int MID_TH; ///> Mid bin in TH dimension

    int MAX_X; ///> Extent of the X dimension 
    int MIN_X; ///> Extent of the X dimension
    int MAX_Y; ///> Extent of the Y dimension
    int MIN_Y; ///> Extent of the Y dimension
    int MAX_TH; ///> Extent of the TH dimension
    int MIN_TH; ///> Extent of the TH dimension
    
    double originX_; ///> Center of the grid in world coordinates 
    double originY_; ///> Center of the grid in world coordinates 

    PoseSpace ps_; ///> This contains the node ids of nodes at each corresponding bin

    PoseSpaceIndex poseToIndex(double x, double y, double th) const;
    int getVertexCountAt_(PoseSpaceIndexType binX, PoseSpaceIndexType binY, PoseSpaceIndexType binTH) const; 
    void insertVertex_(PoseSpaceIndexType binX,PoseSpaceIndexType binY, PoseSpaceIndexType binTH, NodeIdType nodeId);
    public:
	SpatialHash(){};
        SpatialHash(double originX, double originY, int binsX, int binsY, int binsTH, Range rangeX, Range rangeY, Range rangeTH);
        void insertVertex(double x,double y, double th, NodeIdType nodeId);
        int getVertexCountAt(PoseSpaceIndex ind) const; 
	const std::vector<NodeIdType>& getNodeIdVectorAt(double x, double y, double th) const;
	
	void dump();
};

};