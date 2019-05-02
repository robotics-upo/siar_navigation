#ifndef RRTNODE__H__
#define RRTNODE__H__
#include "NodeState.hpp"
struct RRTNode
{
//   int id;
  NodeState st;
  double command_lin; //parent to child command
  double command_ang;
  double cost;
  RRTNode *parent;
  std::vector<RRTNode*> children;
  //std::list<RRTNode*> children;
  RRTNode()
  {
    parent = NULL;
    dead = false;
  }
  bool dead;
};
  
  
#endif