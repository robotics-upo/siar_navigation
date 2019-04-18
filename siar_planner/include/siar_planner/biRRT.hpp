#ifndef _biRRT_HPP_
#define _biRRT_HPP_

#include "siar_planner/Planner.hpp"
#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class biRRT:public Planner
{
public:
  biRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~biRRT();
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path); 
  
  virtual visualization_msgs::Marker getGraphMarker();
  
  std::list<RRTNode *> tree1;
  std::list<RRTNode *> tree2;
  
  RRTNode *q_final_1 = NULL;
  RRTNode *q_final_2 = NULL;

  virtual int getGraphSize() {
    return tree1.size() + tree2.size();
  }
  
protected:
  biRRT();
  
  virtual void clear();
    
  RRTNode* areConnected(NodeState st, bool direct);
  bool got_connected = false;
  
  //new functions
  RRTNode *getNearestNode(NodeState q_rand, bool primary_tree);
  void expandNode(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool direct = true);
  virtual std::list<RRTNode> getPath();
  void expandNearestNodes();
};

biRRT::biRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh, pnh)
{
  // ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K);
}

biRRT::~biRRT(){
  for (auto n:tree1) {
    delete n;
  }
  for (auto n:tree2) {
    delete n;
  }
}


inline void biRRT::clear()
{
  tree1.clear();
  tree2.clear();
  q_final_1 = NULL;
  q_final_2 = NULL;
  got_connected = false;
}


double biRRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path){

  clear();
  if (!m.isInit()) {
    ROS_ERROR("biRRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }
  
  start_node.st = start;
  tree1.push_back(new RRTNode(start_node)); 
  goal_node.st = goal;
  tree2.push_back(new RRTNode(goal_node));
  
  double ret_val = -1.0; 
  int relax = 0;
  
  while (relax < n_rounds && !got_connected) {
    int cont = 0; 
    while (cont < n_iter && !got_connected) { // n_iter Max. number of nodes to expand for each round
      NodeState q_rand;
      if (!(cont%samp_goal_rate == 0)){
	      q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
      	RRTNode *q_near = getNearestNode(q_rand, true);  
        expandNode(q_rand, q_near, relax, true);
      	if(!got_connected){
	       q_near = getNearestNode(q_rand, false);
	       expandNode(q_rand, q_near, relax, false);
	      }
      }
      else{
	      double dist = std::numeric_limits<double>::infinity();
	      RRTNode *q_closest1 = NULL;  
	      RRTNode *q_closest2 = NULL;
	      double new_dist;
        for (auto n1: tree1){ 
          for (auto n2: tree2){
            new_dist = sqrt(pow(n1->st.state[0] - n2->st.state[0],2) + pow(n1->st.state[1] - n2->st.state[1],2)); 
            if (new_dist < dist){
              q_closest1 = n1; 
              q_closest2 = n2; 
              dist = new_dist;
            }
          }
        }
        expandNode(q_closest2->st, q_closest1, relax, true); 
        if(!got_connected){
          expandNode(q_closest1->st, q_closest2, relax, false); 
        }
      }     
      cont++;
    }
    
    if(got_connected){ 
      path = getPath(); 
      ret_val = 1;
      ROS_INFO("Iteration %d. Solution found", relax);
    }
    else{ 
      m.decreaseWheels(wheel_decrease, last_wheel);
      relax++;
      ROS_ERROR("biRRT::resolve -->  could not find a path -->  starting new iteration");    
    }
  }
  std::cout << "Numero de nodos totales: " << tree1.size()+tree2.size() <<std::endl;
  std::cout << "Numero de nodos en path: " << path.size() <<std::endl;
  return ret_val; 
}


RRTNode* biRRT::getNearestNode(NodeState q_rand, bool primary_tree) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  if (primary_tree){
    for (auto n: tree1){ 
      new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
        q_near = n; 
        dist = new_dist;
      }
    } 
  }
  else{
    for (auto n: tree2){ 
      new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
	      q_near = n; 
        dist = new_dist;
      }
    } 
  } 
  return q_near;
}

void biRRT::expandNode(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool direct){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;  
  for (int i = 0; i < K; i++) {
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    double cost;
    if (direct){
      cost = m.integrate(st, command, delta_t, relaxation_mode >= 1); // If relaxation_mode >= 1 --> allow two wheels
      if (cost < 0.0) {
        // 	std::cout << "Colision " <<std::endl;
      }
      else{
        is_new_node = true;
        new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
        if (new_dist<dist) {
          q_new.st = st; 
          q_new.command_lin = command.linear.x;
          q_new.command_ang = command.angular.z;
          dist = new_dist; 
          q_new.cost = cost;
        }
      }
    }
    else{
      cost = m.integrate(st, command, -(delta_t), relaxation_mode >= 1); 
      if (cost < 0.0) {
        // 	std::cout << "Colision " <<std::endl;
      }
      else{
        is_new_node = true;
        new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
        if (new_dist<dist) {
          q_new.st = st; 
          q_near->command_lin = command.linear.x; 
          q_near->command_ang = command.angular.z;
          dist = new_dist;
          q_new.cost = cost;
        }
      }
    }  
  }
  if (is_new_node){
    RRTNode *q_closest = areConnected(q_new.st, direct); 
    if(direct){
      q_new.parent = q_near;
      // q_new.cost += q_near->cost; // Accumulate the parent cost 
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree1.push_back(new_node);
      if(got_connected){
        q_final_1 = new_node;
        q_final_2 = q_closest;
      }
    }     
    else{
      q_near->command_lin = q_new.command_lin;
      q_near->command_ang = q_new.command_ang;
      q_new.command_lin = 0;
      q_new.command_ang = 0;
      q_new.parent = q_near;
      // q_new.cost += q_near->cost; // Accumulate the parent cost 
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree2.push_back(new_node);
      if(got_connected){
        q_final_2 = new_node;
        q_final_1 = q_closest;
      }      
    }
  }
}


RRTNode* biRRT::areConnected(NodeState st, bool direct) {
  double dist = std::numeric_limits<double>::infinity();
  RRTNode *q_closest = NULL;  
  double new_dist;
  if(direct){
    for (auto n: tree2){ 
      new_dist = sqrt(pow(st.state[0] - n->st.state[0],2) + pow(st.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
	      q_closest = n; 
        dist = new_dist;
      }
    }  
  }
  else{
    for (auto n: tree1){ 
      new_dist = sqrt(pow(st.state[0] - n->st.state[0],2) + pow(st.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
      	q_closest = n; 
        dist = new_dist;
      }
    }
  }
  got_connected = (fabs(st.state[0] - q_closest->st.state[0]) < goal_gap_m) && (fabs(st.state[1]-q_closest->st.state[1]) < goal_gap_m) && (fabs(st.state[2] - q_closest->st.state[2]) < goal_gap_rad);
  return q_closest;	  
}


std::list<RRTNode> biRRT::getPath(){
  std::cout << "Nodos en arbol de start: " << tree1.size() << std::endl;
  std::cout << "Nodos en arbol de goal: " << tree2.size() << std::endl;
  std::list<RRTNode> path;
  RRTNode* current_node = q_final_1;
  path.push_front(*current_node); 
  while (current_node->parent != NULL) {  
    current_node = current_node->parent;
    path.push_front(*current_node);
  }
  current_node = q_final_2;
  path.push_front(*current_node);
  while (current_node->parent != NULL) {  
    current_node = current_node->parent;
    path.push_back(*current_node);
  }
  return path;
}


visualization_msgs::Marker biRRT::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "birrt";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::POINTS;
  m.scale.x = 0.05;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  color.g = 0;
  color.a = 1.0;
  geometry_msgs::Point p1;
  for (auto n : tree1){  
    color.r = 0;
    color.b = 1.0;
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    m.points.push_back(p1); 
    m.colors.push_back(color);
  }
  for (auto n : tree2){  
    color.r = 1.0;
    color.b = 0;
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    m.points.push_back(p1); 
    m.colors.push_back(color);   
  }
  m.lifetime = ros::Duration(2);
  return m;
}

#endif
