#ifndef _tbiRRT_HPP_
#define _tbiRRT_HPP_

#include "siar_planner/Planner.hpp"
#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class tbiRRT:public Planner
{
public:
  tbiRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~tbiRRT();
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path); 
  
  virtual visualization_msgs::Marker getGraphMarker();
  
  double getDeltaT() const {return delta_t;}
  
  std::list<RRTNode *> tree1;
  std::list<RRTNode *> tree2;
  RRTNode *q_final_1 = NULL;
  RRTNode *q_final_2 = NULL;

  virtual int getGraphSize() {
    return tree1.size() + tree2.size();
  }
  
protected:
  virtual void clear();
  
  RRTNode* areConnected(NodeState st, bool direct);
  bool got_connected = false;
  
  RRTNode goal_node;
  RRTNode start_node;
   
  RRTNode *getNearestNode(NodeState q_rand, bool primary_tree);
  RRTNode *getNearestNode1(NodeState q_rand);
  void expandNode1(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool direct = true);
  void expandNode2(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool direct = true);
  std::list<RRTNode> getPath();
  void expandNearestNodes();

  Transition testTrans1;
  // Transition testTrans2;

  double Temp_1,Temp_2,Temp_init; //(cost_initial - cost_goal)/2
  double cost_wheels_Qnew1, cost_wheels_Qnew2;
  double cost_Qnear, cost_Qnew, cost_wheels_Qnew;

  ros::Publisher tree1_pub, tree2_pub;
  visualization_msgs::Marker tree1Marker, tree2Marker;
  geometry_msgs::Point ptree1, ptree2;
};

// tbiRRT::tbiRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh, pnh),testTrans1(nh,pnh),testTrans2(nh,pnh){
tbiRRT::tbiRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh, pnh),testTrans1(nh,pnh) {
//   ROS_INFO("K_normal = %f \t alfa: %f \t nFailmax: %f \t Temp_init: %f \t", K_normal,alfa,nFailmax,Temp_init); 
  tree1_pub = nh.advertise<visualization_msgs::Marker>("tree1_marker", 2, true);
  tree2_pub = nh.advertise<visualization_msgs::Marker>("tree2_marker", 2, true);
}

tbiRRT::~tbiRRT(){
  for (auto n:tree1) {
    delete n;
  }
  for (auto n:tree2) {
    delete n;
  }
}


inline void tbiRRT::clear()
{
  tree1.clear();
  tree2.clear();
  q_final_1 = NULL;
  q_final_2 = NULL;
  got_connected = false;
}


double tbiRRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path){
  
  clear();
  if (!m.isInit()) {
    ROS_ERROR("tbiRRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }

  RRTNode start_node, goal_node;
  geometry_msgs::Twist command_init=m.generateRandomCommand(); 

  start_node.st = start;
  goal_node.st = goal;
  start_node.cost = m.integrateTransition(start_node.st, command_init, delta_t);
  goal_node.cost = m.integrateTransition(goal_node.st, command_init, -(delta_t));
  tree1.push_back(new RRTNode(start_node)); 
  tree2.push_back(new RRTNode(goal_node));
  
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
  
  
  tree2Marker.header.frame_id = this->m.getFrameID();
  tree2Marker.header.stamp = ros::Time::now();
  tree2Marker.pose.orientation.w = 1.0;
  tree2Marker.id = 0;
  tree2Marker.type = visualization_msgs::Marker::POINTS;
  tree2Marker.scale.x = 0.05;
  // visualization_msgs::Marker::_color_type color;
  tree2Marker.action = visualization_msgs::Marker::ADD;
  tree2Marker.color.r=1.0;
  tree2Marker.color.a=1.0;
  tree2Marker.color.g=0.2;
  tree2Marker.color.b=0.2;
  tree2Marker.points.clear();
  

  while (relax < n_rounds && !got_connected) {
    int cont = 0; 
    while (cont < n_iter && !got_connected) { // n_iter Max. number of nodes to expand for each round
      NodeState q_rand;
      if (!(cont%samp_goal_rate == 0)){
	      q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
	      RRTNode *q_near = getNearestNode(q_rand, true); 
        expandNode1(q_rand, q_near, relax, true);         
	      if(!got_connected){                              
	        q_near = getNearestNode(q_rand, false);       
          expandNode2(q_rand, q_near, relax, false);      
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
        expandNode1(q_closest2->st, q_closest1, relax, true); 
        if(!got_connected){
          expandNode2(q_closest1->st, q_closest2, relax, false); 
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
      ROS_ERROR("tbiRRT::resolve -->  could not find a path -->  starting new iteration");    
    }
  }
  std::cout << "Totald Nodes: " << tree1.size()+tree2.size() <<std::endl;
  std::cout << "Nodes Path: " << path.size() <<std::endl;
  return ret_val; 
}


RRTNode* tbiRRT::getNearestNode(NodeState q_rand, bool primary_tree) {
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

RRTNode* tbiRRT::getNearestNode1(NodeState q_rand) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  for (auto n: tree1){ 
    new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    if (new_dist < dist) {
      q_near = n; 
      dist = new_dist;
    }
  } 
  for (auto n: tree2){ 
    new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    if (new_dist < dist) {
      q_near = n; 
      dist = new_dist;
    }
  }   
  return q_near;
}


void tbiRRT::expandNode1(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool direct){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  bool ret_transition1 = false; 
  for (int i = 0; i < K; i++) {
    ret_transition1 = false;
    is_new_node = false;
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    cost_wheels_Qnew1 = m.integrateTransition(st, command, delta_t); // If relaxation_mode >= 1 --> allow two wheels
    RRTNode q_try = q_new;
    q_try.st = st;
    q_try.cost = cost_wheels_Qnew1;
    if (cost_wheels_Qnew1 < 0.0) {
      continue;
    }
    ret_transition1 = testTrans1.transitionTest(*q_near, q_try);
    if(ret_transition1){
	    is_new_node = true;
	    new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
      if (new_dist<dist) {
        q_new.st = st; 
        q_new.command_lin = command.linear.x;
        q_new.command_ang = command.angular.z;
        dist = new_dist;
        q_new.cost = cost_wheels_Qnew1; 
      }
    }
  }
  if (is_new_node){
    ptree1.x = q_new.st.state[0];
    ptree1.y = q_new.st.state[1];
    tree1Marker.ns = "tree1M";
    tree1Marker.points.push_back(ptree1);
    tree1Marker.lifetime = ros::Duration(3);
    tree1_pub.publish(tree1Marker); 


    RRTNode *q_closest = areConnected(q_new.st, direct);   
    if(direct){
      q_new.parent = q_near;
      // q_new.cost += q_near->cost;
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree1.push_back(new_node);
      if(got_connected){
	      q_final_1 = new_node;
	      q_final_2 = q_closest;
        // q_final_1->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_2->cost = q_closest->cost;
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
        // q_final_1->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_2->cost = q_closest->cost;
      }      
    }
  }
}        


void tbiRRT::expandNode2(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool direct){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  bool ret_transition2 = false; 

  for (int i = 0; i < K; i++) {
    ret_transition2 = false;
    is_new_node = false;
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    cost_wheels_Qnew2 = m.integrateTransition2(st, command, -(delta_t)); 
    RRTNode q_try = q_new;
    q_try.st = st;
    q_try.cost = cost_wheels_Qnew2;
      if (cost_wheels_Qnew2 < 0.0) {
        continue;
      }
      ret_transition2 = testTrans1.transitionTest(*q_near, q_try);
      if(ret_transition2){
        is_new_node = true;
        new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
        if (new_dist<dist) {
          q_new.st = st; 
          q_near->command_lin = command.linear.x;
          q_near->command_ang = command.angular.z;
          dist = new_dist;
          q_new.cost = cost_wheels_Qnew2; 
        }
      }
  }
  if (is_new_node){
    ptree2.x = q_new.st.state[0];
    ptree2.y = q_new.st.state[1];
    tree2Marker.ns = "tree2M";
    tree2Marker.points.push_back(ptree2);
    tree2Marker.lifetime = ros::Duration(3);
    tree2_pub.publish(tree2Marker);  

    RRTNode *q_closest = areConnected(q_new.st, direct); 
    if(direct){
      q_new.parent = q_near;
      // q_new.cost += q_near->cost;
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree1.push_back(new_node);
      if(got_connected){
	      q_final_1 = new_node;
	      q_final_2 = q_closest;
        // q_final_1->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_2->cost = q_closest->cost;
      }
    }     
    else{
      q_near->command_lin = q_new.command_lin; 
      q_near->command_ang = q_new.command_ang;
      q_new.command_lin = 0;
      q_new.command_ang = 0;
      q_new.parent = q_near;
      // q_new.cost += q_near->cost;
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree2.push_back(new_node);
      if(got_connected){
	      q_final_2 = new_node;
	      q_final_1 = q_closest;
        // q_final_2->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_1->cost = q_closest->cost;
      }      
    }
  }
}                            


RRTNode* tbiRRT::areConnected(NodeState st, bool direct) {
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
  got_connected = (fabs(st.state[0] - q_closest->st.state[0]) < goal_gap_m) && (fabs(st.state[1]-q_closest->st.state[1]) < goal_gap_m) && 
  (fabs(st.state[2] - q_closest->st.state[2]) < goal_gap_rad);
  return q_closest;	 
}


std::list<RRTNode> tbiRRT::getPath(){ //get path from finished graph
  std::cout << "Nodes start tree: " << tree1.size() << std::endl;
  std::cout << "Nodes goal tree: " << tree2.size() << std::endl;
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


visualization_msgs::Marker tbiRRT::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "tbirrt";
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
