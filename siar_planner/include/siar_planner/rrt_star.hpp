#ifndef _RRT_STAR_HPP_
#define _RRT_STAR_HPP_

#include "siar_planner/rrt.hpp"

#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"
#include "siar_planner/metrica.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class RRTStar:public RRT
{
public:
  RRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~RRTStar();

  
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);    
  
  
protected:
  // RRTStar();
  void addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0, RRTNode *parent = NULL); 
  
  //new functions
  void expandNode(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0);

  // rewire
  void rewire(RRTNode* q_new, RRTNode* q_near, double radius);

  virtual void clear();

  double cost_Qnear, cost_Qnew, cost_wheels_Qnew;

  Metrica metrica;

};

RRTStar::RRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh):RRT(nh,pnh), metrica(nh,pnh)
{
  // tree1_pub = nh.advertise<visualization_msgs::Marker>("tree1_marker", 2, true);
}

RRTStar::~RRTStar()
{
  clear();
}


inline void RRTStar::clear()
{
  RRT::clear();
  // for (auto n:nodes) {
  //   delete n;
  // }
  // nodes.clear();
  // got_to_goal = false;
}


double RRTStar::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  clear(); 

  if (!m.isInit()) {
    ROS_ERROR("RRTStar::resolve --> The model has not been initialized --> could not calculate a path");
    return -2.0;
  }

  if (m.isCollision(goal) || m.isCollision(start)) {
    ROS_ERROR("RRTStar::resolve --> The goal or the starting point is not valid");
    return -3.0;
  }
  
  RRTNode start_node;
  geometry_msgs::Twist command_init=m.generateRandomCommand(); 

  start_node.st = start;
  start_node.cost = m.integrate(start_node.st, command_init, delta_t);
  nodes.push_back(new RRTNode(start_node)); 
  goal_node.st = goal;
  
  double ret_val = -1.0; 
  int relax = 0;
  
  tree1Marker.header.frame_id = this->m.getFrameID();
  tree1Marker.header.stamp = ros::Time::now();
  tree1Marker.pose.orientation.w = 1.0;
  tree1Marker.id = 0;
  tree1Marker.type = visualization_msgs::Marker::POINTS;
  tree1Marker.scale.x = 0.05;
  // visualization_msgs::Marker::_color_type color;
  tree1Marker.action = visualization_msgs::Marker::ADD;
  tree1Marker.color.b=1.0;
  tree1Marker.color.a=1.0;
  tree1Marker.color.r=0.4;
  tree1Marker.color.g=0.2;
  tree1Marker.points.clear();
  
  while (relax < n_rounds && !got_to_goal){ 
    int cont = 0; 
    while (cont < n_iter && !got_to_goal) { 
    
      NodeState q_rand;
      if (!(cont%samp_goal_rate == 0)){
	      q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
      } else {
      	q_rand = goal_node.st; 
      }     
      RRTNode *q_near = getNearestNode(q_rand);
      if (q_near == NULL){
        q_near = getNearestNode(q_rand, true);  
      }
      expandNode(q_rand, q_near, relax);
      cont++;
    }
    
    if(got_to_goal){ 
      path = getPath(); 
      ret_val = 1;
      ROS_INFO("Iteration %d. Solution found", relax);
    }
    else{   
      // TODO increase or decrease the wheels randomly
      ROS_ERROR("RRTstar::resolve -->  could not find a path -->  starting iteration: %i", relax);    
    }
  }
  std::cout << "Nodes in the graph: " << nodes.size() <<std::endl;
  return ret_val; 
}

void RRTStar::expandNode(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  bool ret_transition = false; 
  bool is_dead = true;
  RRTNode q_try;
  for (int i = 0; i < K; i++) {   //Are generate K different nodes (4) with different STATES to evaluate !!
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand(); //generate a random command of velocity
    cost_wheels_Qnew = m.integrate(st, command, delta_t);
    // RRTNode q_try;
    q_try.st = st;
    q_try.cost = cost_wheels_Qnew;
    if (cost_wheels_Qnew < 0.0){
      continue;
    }

    RRTNode *near_ = getNearestNode(st, true);
    double d = 0.2;
    if (near_ != NULL) {
      d = metrica.metrica3D(near_->st, st);
    }

    if (d > 0.01) {
      is_dead = false;
      is_new_node = true;
      new_dist = metrica.metrica3D(q_rand,st);
      if (new_dist<dist) {
        q_new.st = st; 
        q_new.command_lin = command.linear.x;
        q_new.command_ang = command.angular.z;
        dist = new_dist;
        q_new.cost = cost_wheels_Qnew;   
      }    
    } 
  }
  if (is_new_node){   //check if there is a new node and add it to the graph (unless its the goal)
    ptree1.x = q_new.st.state[0];
    ptree1.y = q_new.st.state[1];
    tree1Marker.ns = "treeStar";
    tree1Marker.points.push_back(ptree1);
    tree1Marker.lifetime = ros::Duration(40);
    tree1_pub.publish(tree1Marker); 

    isGoal(q_new.st);
    if(got_to_goal){
      q_near->children.push_back(&goal_node);
      goal_node.parent = q_near; 
      goal_node.command_lin = q_new.command_lin; 
      goal_node.command_ang = q_new.command_ang;  
      // goal_node.cost = q_new.cost + q_near->cost; // New field:cost
    }
    else{
      q_new.parent = q_near;
      // q_new.cost += q_near->cost; // Accumulate the parent cost 
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      nodes.push_back(new_node);	
    }
  } 
  else if (is_dead) {
    q_near->dead = true;
    RRTNode *curr = q_near->parent;
    int cont = 0;
    for (;curr != NULL && cont < num_delete_parents; cont++, curr = q_near->parent) {
      curr->dead = true;
    }
  }
}

void RRTStar::rewire(RRTNode* q_new, RRTNode* q_near, double radius){
  for (auto n:nodes) {
    if (n != q_near && n != q_new && !n->dead) {
      double d = metrica.metrica3D(n->st, q_new->st);
      if (d < radius) {
        for (auto &command:command_list) {
          auto Qint = q_new->st;
          double cost = m.integrate(Qint, command, delta_t);
          double d_new = metrica.metrica3D(n->st, Qint);

          if (d_new < 0.05 && d_new < d && cost > 0.0 && cost + q_new->cost < n->cost) {
            // Rewire
            if (n->parent != NULL) {
              auto it = std::find(n->parent->children.begin(), n->parent->children.end(), n);
              if (it != n->parent->children.end()) {
                n->parent->children.erase(it);
              }
            }
            n->parent = q_new;
            q_new->children.push_back(n);
            n->cost = cost + q_new->cost;
            break;
          }
        }
      }
    }
  }
}


#endif

