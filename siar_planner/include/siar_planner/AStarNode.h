#ifndef ASTARNODE__H__
#define ASTARNODE__H__
#include "NodeState.hpp"
struct AStarNode
{
  int id;
  NodeState st;
  double command_lin;
  double command_ang;
  std::unordered_map<int,double> neighbors;
  double gScore;
  double fScore;
  double c_plus;
  double c_minus;
  AStarNode *parent;
};
  
  
#endif