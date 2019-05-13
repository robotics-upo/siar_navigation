#ifndef _biRRT_HPP_
#define _biRRT_HPP_

#include "siar_planner/Planner.hpp"
#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"
#include "siar_planner/metrica.hpp"

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

  double getDeltaT() const {return delta_t;}
  
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
    
  bool areConnected(NodeState st, const std::list<RRTNode *> &tree, RRTNode **connection_point);
  bool got_connected = false;
  
  //new functions
  RRTNode *getNearestNode(NodeState q_rand, const std::list<RRTNode*> &tree, bool consider_dead = false);
  void expandNode1(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool justRot = true);
  void expandNode2(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool justRot = true);
  virtual std::list<RRTNode> getPath();
  void expandNearestNodes();

  void getNearestNodeAndExpand(NodeState q, int relax);
  // bool getStraightNode(RRTNode *q_near,NodeState &q_rand_straight, bool relaxation_mode);

  // ros::Publisher tree1_pub, tree2_pub, random_pub;

  Metrica metrica;

  ros::Publisher tree1_pub, tree2_pub;
  visualization_msgs::Marker tree1Marker, tree2Marker;
  geometry_msgs::Point ptree1, ptree2;
  
};

biRRT::biRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh, pnh), metrica(nh,pnh)
{
  // ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K);
  tree1_pub = nh.advertise<visualization_msgs::Marker>("tree1_marker", 2, true);
  tree2_pub = nh.advertise<visualization_msgs::Marker>("tree2_marker", 2, true);
  // random_pub = nh.advertise<visualization_msgs::Marker>("random_marker", 2, true);
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
    return -2.0;
  }
  if (m.isCollisionTransition(goal) || m.isCollisionTransition(goal)) {
    ROS_ERROR("tbiRRT::resolve --> The goal or the starting point is not valid");
    return -3.0;
  }
  
  start_node.st = start;
  tree1.push_back(new RRTNode(start_node)); 
  goal_node.st = goal;
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
      	getNearestNodeAndExpand(q_rand, relax);
      }
      else{
	      double dist = std::numeric_limits<double>::infinity();
	      RRTNode *q_closest1 = NULL;  
	      RRTNode *q_closest2 = NULL;
	      double new_dist;
        for (auto n1: tree1){ 
          for (auto n2: tree2){
            new_dist = metrica.metrica3D(*n1,*n2);
            if (new_dist < dist){
              q_closest1 = n1; 
              q_closest2 = n2; 
              dist = new_dist;
            }
          }
        }
          expandNode1(q_closest2->st, q_closest1, relax, false); 
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
      ROS_ERROR("biRRT::resolve -->  could not find a path -->  starting iteration: %i", relax);    
    }
  }
  std::cout << "Total Nodes: " << tree1.size()+tree2.size() <<std::endl;
  std::cout << "Nodes tree1= " << tree1.size() << "  ;  Nodes tree2= " << tree2.size() << std::endl;
  std::cout << "Nodes in Path: " << path.size() <<std::endl;
  return ret_val; 
}



RRTNode* biRRT::getNearestNode(NodeState q_rand, const std::list<RRTNode*> &tree, bool consider_dead) {
  RRTNode *q_near = NULL;
  double dist = std::numeric_limits<double>::infinity();
  double new_dist;
  for (auto n: tree){
      // new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2));
      new_dist = metrica.metrica3D(q_rand,n->st);
      if (new_dist < dist && (!n->dead || consider_dead)) {
	      q_near = n;
        dist = new_dist;
      }
  }
  return q_near;
}

void biRRT::getNearestNodeAndExpand(NodeState q, int relax) {
  RRTNode *q1 = getNearestNode(q, tree1);
  RRTNode *q2 = getNearestNode(q, tree2);
  if (q1 == NULL && q2 == NULL)
    return;

  if (q2 == NULL) {

    expandNode1(q, q1, relax, false);
  } else if (q1 == NULL) {

    expandNode2(q, q2, relax, false);
  } else if ( metrica.metrica3D(q1->st, q) < metrica.metrica3D(q2->st,q)) {
    expandNode1(q, q1, relax, false);
  } else {
    expandNode2(q, q2, relax, false);
  }
}


void biRRT::expandNode1(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool justRot){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;  
  bool is_dead = true;
  
  for (int i = 0; i < K; i++) { //integrate different command evaluator from the same q_near
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand(); 
    double cost;
    cost = m.integrate(st, command, delta_t, relaxation_mode >= 1); // If relaxation_mode >= 1 --> allow two wheels
    if (cost < 0.0) {
    }
    else{
      RRTNode *near_ = getNearestNode(st, tree1, true);
      double d = 0.2;
      
      if (near_ != NULL) {
        d = metrica.metrica3D(near_->st, st);
      }
      if (d > same_node_dist) {
        
        is_dead = false;
        is_new_node = true;
        new_dist = metrica.metrica3D(q_rand,st);
        if (new_dist<dist) {
          q_new.st = st; 
          q_new.command_lin = command.linear.x;
          q_new.command_ang = command.angular.z;
          dist = new_dist; 
          q_new.cost = cost;
        }
      }
    }
  }
  if (is_new_node){
    ptree1.x = q_new.st.state[0];
    ptree1.y = q_new.st.state[1];
    tree1Marker.ns = "tree1M";
    tree1Marker.points.push_back(ptree1);
    tree1Marker.lifetime = ros::Duration(40);
    tree1_pub.publish(tree1Marker); 
    
    q_near->command_lin = q_new.command_lin;
    q_near->command_ang = q_new.command_ang;
    q_new.command_lin = 0;
    q_new.command_ang = 0;
    q_new.parent = q_near;
    RRTNode *new_node = new RRTNode(q_new); 
    q_near->children.push_back(new_node);
    tree1.push_back(new_node);
    if(areConnected(q_new.st, tree2, &q_final_2)){
	    q_final_1 = new_node;
      got_connected = true;
    }
  } else if (is_dead) {

    q_near->dead = true;
    RRTNode *curr = q_near->parent;
    int cont = 0;

    for (;curr != NULL && cont < num_delete_parents; cont++, curr = q_near->parent) {
      curr->dead = true;
    }
  }
}


void biRRT::expandNode2(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool justRot){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;  
  bool is_dead = true;
  
  for (int i = 0; i < K; i++) { //integrate different command evaluator from the same q_near
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand(); 
    double cost;
    cost = m.integrate(st, command, -(delta_t), relaxation_mode >= 1); 
    if (cost < 0.0) {
    }
    else{
      RRTNode *near_ = getNearestNode(st, tree2, true);
      double d = 0.2;
      
      if (near_ != NULL) {
        d = metrica.metrica3D(near_->st, st);
      }
      if (d > same_node_dist) {
        is_dead = false;

        is_new_node = true;
        new_dist = metrica.metrica3D(q_rand,st);
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
    ptree2.x = q_new.st.state[0];
    ptree2.y = q_new.st.state[1];
    tree2Marker.ns = "tree2M";
    tree2Marker.points.push_back(ptree2);
    tree2Marker.lifetime = ros::Duration(40);
    tree2_pub.publish(tree2Marker);  

    q_near->command_lin = q_new.command_lin;
    q_near->command_ang = q_new.command_ang;
    q_new.command_lin = 0;
    q_new.command_ang = 0;
    q_new.parent = q_near;
    RRTNode *new_node = new RRTNode(q_new); 
    q_near->children.push_back(new_node);
    tree2.push_back(new_node);
    if(areConnected(q_new.st, tree1, &q_final_1)){
	    q_final_2 = new_node;
	    // q_final_1 = getNearestNode(q_new.st, tree1);
      got_connected = true;
    }    
  } else if (is_dead) {

    q_near->dead = true;
    RRTNode *curr = q_near->parent;
    int cont = 0;

    for (;curr != NULL && cont < num_delete_parents; cont++, curr = q_near->parent) {
      curr->dead = true;
    }
  }
}

bool biRRT::areConnected(NodeState st, const std::list<RRTNode *> &tree, RRTNode **connection_point) {
  bool ret_val = false;
  *connection_point = NULL;
  // ROS_INFO("Testing conection: %f %f %f", st.state[0], st.state[1], st.state[2]);
  for (auto n: tree) {
        // ROS_INFO("Distance: %f  %f", fabs(st.state[0] - n->st.state[0]), fabs(st.state[1]-n->st.state[1]));
      ret_val = (fabs(st.state[0] - n->st.state[0]) < goal_gap_m) && (fabs(st.state[1]-n->st.state[1]) < goal_gap_m) &&
  (metrica.getAngDist(st.state[2], n->st.state[2]) < goal_gap_rad);
      if (ret_val) {

        ROS_INFO("Are connected. New val: %d", ret_val);
        *connection_point = n;
        break;
      }

    
  }
  

  return ret_val;
}


std::list<RRTNode> biRRT::getPath(){
  std::cout << "Nodes in start tree: " << tree1.size() << std::endl;
  std::cout << "Nodes in goal tree: " << tree2.size() << std::endl;
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
  geometry_msgs::Point p1, p2;
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
    p2.x = n->st.state[0];
    p2.y = n->st.state[1];
    m.points.push_back(p2); 
    m.colors.push_back(color);   
  }
  m.lifetime = ros::Duration(2);
  return m;
}

#endif
