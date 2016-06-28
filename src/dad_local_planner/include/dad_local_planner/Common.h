#pragma once

#include <cmath>
#include <utility>

namespace dad_local_planner
{
  

  

class NodeComparator
{
public:
  bool operator() (const Node& lhs, const Node&rhs) const
  {
    if ( std::abs(lhs.totalCost - rhs.totalCost) < 0.1 ) // changed from 0.1 to 0.01
    {
      //std::cout << " Costs are equal. Trying to tie break " << std::endl;
      
      if ( lhs.hCostFake > rhs.hCostFake )
      {
        return true; // rhs is better
      }
      else
      {
        return false; // lhs is better
      }
     
      
    /* 
      if ( rhs.pathPosition > lhs.pathPosition )
      {
        return true;
      }
      else
      {
        return false;
      }
      */
      
      
      
    }
    if ( lhs.totalCost > rhs.totalCost )
    {
      return true; // rhs is better
    }
    else
    {
      return false; // lhs is better
    }
  }
};

};