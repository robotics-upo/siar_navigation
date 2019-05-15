#ifndef _tRRT_HPP_
#define _tRRT_HPP_

#include "siar_planner/Planner.hpp"

#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"
#include "siar_planner/Transition.hpp"
#include "siar_planner/metrica.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class tRRT:public Planner
{
public:
  tRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~tRRT();
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);    
  
  virtual visualization_msgs::Marker getGraphMarker();
  
  virtual int getGraphSize() {
    return nodes.size();
  }

  std::list<RRTNode *> nodes;

protected:
  // tRRT();
  void addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0, RRTNode *parent = NULL); 
  virtual void clear();
  
  //new functions
  RRTNode *getNearestNode(NodeState q_rand, bool consider_dead = false);
  void expandNode(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0);
  virtual std::list<RRTNode> getPath();

  Transition testTrans;

  double K_normal, Temp_init, Temp; 
  int alfa , nFail, nFailmax ;
  double cost_Qnear, cost_Qnew, cost_wheels_Qnew;

  ros::Publisher tree1_pub;
  visualization_msgs::Marker tree1Marker;
  geometry_msgs::Point ptree1;

  Metrica metrica;

};

tRRT::tRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh,pnh),testTrans(nh,pnh), metrica(nh,pnh)
{
  tree1_pub = nh.advertise<visualization_msgs::Marker>("tree1_marker", 2, true);

//   ROS_INFO("K_normal = %f \t alfa: %f \t nFailmax: %f \t Temp_init: %f \t", K_normal,alfa,nFailmax,Temp_init); 
}

tRRT::~tRRT()
{
  clear();
}


inline void tRRT::clear()
{
  for (auto n:nodes) {
    delete n;
  }
  nodes.clear();
  got_to_goal = false;
}


double tRRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  clear(); 

  if (!m.isInit()) {
    ROS_ERROR("tRRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -2.0;
  }

  if (m.isCollisionTransition(goal) || m.isCollisionTransition(goal)) {
    ROS_ERROR("tbiRRT::resolve --> The goal or the starting point is not valid");
    return -3.0;
  }
  
  
  RRTNode start_node;
  geometry_msgs::Twist command_init=m.generateRandomCommand(); 

  start_node.st = start;
  start_node.cost = m.integrateTransition(start_node.st, command_init, delta_t);
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
      }
      else{
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
      m.decreaseWheels(wheel_decrease, last_wheel);
      relax++;
      ROS_ERROR("tRRT::resolve -->  could not find a path -->  starting iteration: %i", relax);    
    }
  }
  std::cout << "Nodes in the graph: " << nodes.size() <<std::endl;
  return ret_val; 
}


RRTNode* tRRT::getNearestNode(NodeState q_rand, bool consider_dead) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  for (auto n: nodes){ 
    // new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    new_dist = metrica.metrica3D(q_rand,n->st);
    
    if (new_dist < dist  && (!n->dead || consider_dead)) {
      q_near = n; 
      dist = new_dist;
    }    
  }
  return q_near;
}


void tRRT::expandNode(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  bool ret_transition = false; 
  bool is_dead = true;
  RRTNode q_try;
  for (int i = 0; i < K; i++) {   //Are generate K different nodes (4) with different STATES to evaluate !!
    // ret_transition = false;
    // is_new_node = false;
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand(); //generate a random command of velocity
    cost_wheels_Qnew = m.integrateTransition(st, command, delta_t);
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
      ret_transition = testTrans.transitionTest(*q_near, q_try);
      if (ret_transition){      
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
  }
  if (is_new_node){   //check if there is a new node and add it to the graph (unless its the goal)
    ptree1.x = q_new.st.state[0];
    ptree1.y = q_new.st.state[1];
    tree1Marker.ns = "tree1M";
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


std::list<RRTNode> tRRT::getPath(){
  std::list<RRTNode> path;
  RRTNode* current_node = &goal_node;
  path.push_front(*current_node);
  while (current_node->parent != NULL) {  
    current_node = current_node->parent;
    path.push_front(*current_node);
  }
  return path;
}

visualization_msgs::Marker tRRT::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::POINTS;
  m.scale.x = 0.05;
  visualization_msgs::Marker::_color_type color;
  color.r = 1.0;
  color.b = 0;
  color.g = 0;
  color.a = 1.0;
  double color_step = 1.0/(double)nodes.size();
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  for (auto n : nodes){  
    auto new_color = color;
    new_color.r -= color_step;
    new_color.b += color_step;
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    m.points.push_back(p1); 
    m.colors.push_back(color);
    color = new_color;
  }
  m.lifetime = ros::Duration(2);

  return m;
}

#endif

