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
#include <algorithm>
#include <cmath>
#include <math.h>
#include <vector>

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
  void updateSubtreeCosts(RRTNode* root, double cost_delta);

  virtual void clear();

  double cost_Qnear, cost_Qnew, cost_wheels_Qnew;
  double rewire_radius;
  double safety_weight;

  Metrica metrica;

};

RRTStar::RRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : RRT(nh,pnh), safety_weight(0.5), metrica(nh,pnh)
{
  if (!pnh.getParam("rewire_radius", rewire_radius)) {
    rewire_radius = cellsize_m;
  }
  pnh.param("rrtstar_safety_weight", safety_weight, safety_weight);
  if (!std::isfinite(safety_weight) || safety_weight < 0.0 || safety_weight > 1.0) {
    ROS_WARN("rrtstar_safety_weight must be in [0, 1]; clamping its value");
    safety_weight = std::isfinite(safety_weight)
      ? std::max(0.0, std::min(1.0, safety_weight))
      : 0.5;
  }
  // tree1_pub = nh.advertise<visualization_msgs::Marker>("tree1_marker", 2, true);
}

RRTStar::~RRTStar()
{
  clear();
}


inline void RRTStar::clear()
{
  RRT::clear();
  goal_node.parent = NULL;
  goal_node.children.clear();
  goal_node.cost = 0.0;
  goal_node.total_cost = 0.0;
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
  start_node.st = start;
  start_node.cost = 0.0;
  start_node.total_cost = 0.0;
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
    
      NodeState q_rand = getSampleState(cont);
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
  struct Candidate {
    NodeState state;
    double command_lin;
    double command_ang;
    double edge_cost;
    double distance;
    double wheel_cost;
  };
  std::vector<Candidate> candidates;
  if (K > 0) {
    candidates.reserve(static_cast<std::size_t>(K));
  }
  double min_distance = std::numeric_limits<double>::infinity();
  double max_distance = 0.0;
  bool ret_transition = false; 
  bool is_dead = true;
  RRTNode q_try;
  for (int i = 0; i < K; i++) {   //Are generate K different nodes (4) with different STATES to evaluate !!
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand(); //generate a random command of velocity
    cost_wheels_Qnew = m.integrate(st, command, delta_t);
    if (cost_wheels_Qnew < 0.0){
      continue;
    }
    const double wheel_cost = m.costWheelsQnear(
      st.state[0], st.state[1], st.state[2]);
    // RRTNode q_try;
    q_try.st = st;
    q_try.cost = cost_wheels_Qnew + wheel_cost;

    RRTNode *near_ = getNearestNode(st, true);
    double d = 0.2;
    if (near_ != NULL) {
      d = metrica.metrica3D(near_->st, st);
    }

    if (d > 0.01) {
      Candidate candidate;
      candidate.state = st;
      candidate.command_lin = command.linear.x;
      candidate.command_ang = command.angular.z;
      candidate.edge_cost = q_try.cost;
      candidate.distance = metrica.metrica3D(q_rand, st);
      if (!std::isfinite(candidate.distance)) {
        continue;
      }
      candidate.wheel_cost = wheel_cost;
      is_dead = false;
      min_distance = std::min(min_distance, candidate.distance);
      max_distance = std::max(max_distance, candidate.distance);
      candidates.push_back(candidate);
    } 
  }
  bool is_new_node = !candidates.empty();
  if (is_new_node) {
    const double distance_range = max_distance - min_distance;
    double best_score = std::numeric_limits<double>::infinity();
    for (const Candidate& candidate : candidates) {
      const double distance_score = distance_range > 0.0
        ? (candidate.distance - min_distance) / distance_range
        : 0.0;
      const double wheel_score = std::max(0.0, std::min(100.0, candidate.wheel_cost)) / 100.0;
      const double score =
        (1.0 - safety_weight) * distance_score + safety_weight * wheel_score;
      if (score < best_score) {
        best_score = score;
        q_new.st = candidate.state;
        q_new.command_lin = candidate.command_lin;
        q_new.command_ang = candidate.command_ang;
        q_new.cost = candidate.edge_cost;
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
      goal_node.cost = q_new.cost;
      goal_node.total_cost = q_near->total_cost + q_new.cost;
    }
    else{
      q_new.parent = q_near;
      q_new.total_cost = q_near->total_cost + q_new.cost;
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      nodes.push_back(new_node);
      rewire(new_node, q_near, rewire_radius);
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
      bool creates_cycle = false;
      for (RRTNode* ancestor = q_new; ancestor != NULL; ancestor = ancestor->parent) {
        if (ancestor == n) {
          creates_cycle = true;
          break;
        }
      }
      if (creates_cycle) {
        continue;
      }

      double d = metrica.metrica3D(n->st, q_new->st);
      if (d < radius) {
        for (auto &command:command_list) {
          auto Qint = q_new->st;
          double cost = m.integrate(Qint, command, delta_t);
          if (cost <= 0.0) {
            continue;
          }
          double d_new = metrica.metrica3D(n->st, Qint);

          if (d_new >= 0.05 || d_new >= d) {
            continue;
          }
          const double edge_cost = cost + m.costWheelsQnear(
            Qint.state[0], Qint.state[1], Qint.state[2]);
          if (q_new->total_cost + edge_cost < n->total_cost) {
            // Rewire
            if (n->parent != NULL) {
              auto it = std::find(n->parent->children.begin(), n->parent->children.end(), n);
              if (it != n->parent->children.end()) {
                n->parent->children.erase(it);
              }
            }
            n->parent = q_new;
            q_new->children.push_back(n);
            const double cost_delta = q_new->total_cost + edge_cost - n->total_cost;
            n->cost = edge_cost;
            n->total_cost = q_new->total_cost + edge_cost;
            n->command_lin = command.linear.x;
            n->command_ang = command.angular.z;
            updateSubtreeCosts(n, cost_delta);
            break;
          }
        }
      }
    }
  }
}

void RRTStar::updateSubtreeCosts(RRTNode* root, double cost_delta)
{
  for (RRTNode* child : root->children) {
    child->total_cost += cost_delta;
    updateSubtreeCosts(child, cost_delta);
  }
}


#endif

