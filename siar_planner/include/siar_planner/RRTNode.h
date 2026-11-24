#ifndef RRTNODE__H__
#define RRTNODE__H__
#include "NodeState.hpp"
struct RRTNode
{
//   int id;
  NodeState st;
  double command_lin; //parent to child command
  double command_ang;
  double command_mot = 60; // parent to child command to set width motor
  double cost;
  double total_cost;
  RRTNode *parent;
  std::vector<RRTNode*> children;

  RRTNode(): command_lin(0.0), command_ang(0.0), cost(0.0), total_cost(0.0), parent(NULL), dead(false)
  {
  }
  bool dead;
};
  
  
#endif