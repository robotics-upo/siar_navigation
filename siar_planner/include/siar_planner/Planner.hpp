#ifndef _PLANNER_HPP_
#define _PLANNER_HPP_


#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class Planner
{
public:
  Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path) = 0;      
  virtual double retCostPath(const std::list< RRTNode >& path);

  SiarModel &getModel() {return m;}

  virtual visualization_msgs::MarkerArray getPathMarker(const std::list< RRTNode >& path);
  
  virtual visualization_msgs::Marker getGraphMarker() = 0;
  
  double getDeltaT() const {return delta_t;}

  virtual int getGraphSize() = 0;

  RRTNode goal_node, start_node;
  std::list<NodeState> randomTree;
  visualization_msgs::Marker randomTreeMarker(const std::list< NodeState > randomTree);
  bool resetMarker(bool &reset);
  bool clean;
  
protected:
  virtual void clear() = 0;
  
  virtual void isGoal(NodeState st);
  bool got_to_goal = false;
  
  //new functions
  NodeState getRandomState(double max_x, double max_y, double max_yaw, double min_x, double min_y, double min_yaw);
  virtual std::list<RRTNode> getPath() = 0;
    
  int K, n_iter, n_rounds;
  double delta_t, cellsize_m, cellsize_rad;
  double wheel_decrease, last_wheel;
  double max_x, max_y, max_yaw, min_x, min_y, min_yaw; 
  
  SiarModel m;
  double goal_gap_m, goal_gap_rad;
  
  int samp_cont, samp_goal_rate; 
  
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;

  ros::Publisher random_pub;
  visualization_msgs::Marker randomMarker;
  geometry_msgs::Point ptrandom;
  
};

Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh), gen(rd()), dis(0,1)
{
  if (!pnh.getParam("K", K)) { // Number of random commands
    K = 4;
  }
  
  if (!pnh.getParam("n_iter", n_iter)) {
    n_iter = 1000;
  }
  
  if (!pnh.getParam("n_rounds", n_rounds)) {
    n_rounds = 6;
  }
  
  if (!pnh.getParam("delta_t", delta_t)) {
    delta_t = 0.5;
  }
  
  if (!pnh.getParam("wheel_decrease", wheel_decrease)) {
    wheel_decrease = 0.05;
  }
  
  if (!pnh.getParam("last_wheel", last_wheel)) {
    last_wheel = 0.1;
  }
  
  if (!pnh.getParam("goal_gap_m", goal_gap_m)) {
    goal_gap_m = 0; 
  }
  
  if (!pnh.getParam("goal_gap_rad", goal_gap_rad)) {
    goal_gap_rad = 0; 
  }
  
  if (!pnh.getParam("max_x", max_x)) {  //change thsese default values 
    max_x = 1;
  }
  
  if (!pnh.getParam("max_y", max_y)) {
    max_y = 1;
  }
  
  if (!pnh.getParam("max_yaw", max_yaw)) {
    max_yaw = 1;
  }
  
  if (!pnh.getParam("min_x", min_x)) { 
    min_x = 0;
  }
  
  if (!pnh.getParam("min_y", min_y)) {
    min_y = 0;
  }
  
  if (!pnh.getParam("min_yaw", min_yaw)) {
    min_yaw = 0;
  }
  
  if (!pnh.getParam("samp_goal_rate", samp_goal_rate)) {
    samp_goal_rate = 10;
  }

  if (!pnh.getParam("cellsize_m", cellsize_m)) { 
    cellsize_m = 0.2;
  }
  if (!pnh.getParam("cellsize_rad", cellsize_rad)) {
    cellsize_rad = 0.2;
  }
   
  ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K); 

  random_pub = nh.advertise<visualization_msgs::Marker>("random_marker", 2, true);

}


bool Planner::resetMarker(bool &reset){
    clean = reset;
return clean;
}


NodeState Planner::getRandomState(double max_x, double min_x, double max_y, double min_y, double max_yaw, double min_yaw) {

  if (clean ){
    // ROS_ERROR ("INGRESO A CLEAN");
    randomMarker.points.clear();
    bool clean = false; 
    resetMarker(clean);
  }

  randomMarker.header.frame_id = this->m.getFrameID();
  randomMarker.header.stamp = ros::Time::now();
  randomMarker.pose.orientation.w = 1.0;
  randomMarker.id = 0;
  randomMarker.ns = "random";
  randomMarker.type = visualization_msgs::Marker::POINTS;
  randomMarker.scale.x = 0.02;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  randomMarker.action = visualization_msgs::Marker::ADD;
  randomMarker.color.b=0.2;
  randomMarker.color.a=1.0;
  randomMarker.color.g=1.0;
  randomMarker.color.r=0.2;

  NodeState randomState;
  int index;
   for (int i = 0; i < n_iter; i++){
    randomState.state.resize(3);
    randomState.state[0] = (dis(gen) * (max_x-min_x)) + min_x;  //set random value for each state varible
    randomState.state[1] = (dis(gen) * (max_y-min_y)) + min_y;
    randomState.state[2] = (dis(gen) * (max_yaw-min_yaw)) + min_yaw;

    index = m.m_ce.point2index(randomState.state[0], randomState.state[1]);

    if (m.m_world.data[index] == 127){
      continue;
    }
    else{
      randomTree.push_back(randomState);
      ptrandom.x = randomState.state[0];
      ptrandom.y = randomState.state[1];
      randomMarker.points.push_back(ptrandom);
      random_pub.publish(randomMarker);
      // randomMarker.lifetime= ros::Duration(3);  
      return randomState;
    }
  }
}

void Planner::isGoal(NodeState st) {
  got_to_goal = (fabs(st.state[0] - goal_node.st.state[0]) < goal_gap_m) && (fabs(st.state[1]-goal_node.st.state[1]) < goal_gap_m) &&
            (fabs(st.state[2] - goal_node.st.state[2]) < goal_gap_rad);
}

visualization_msgs::MarkerArray Planner::getPathMarker(const std::list< RRTNode >& path) 
{
  visualization_msgs::MarkerArray ret;
  visualization_msgs::Marker m_aux;

  int cont = 0;
  NodeState pt;
  for (auto it = path.begin(); it != path.end(); it++, cont++) {
    if (cont > 0) { 
      pt = (--it)->st;
      it++;
      if (cont % 5 == 0) { 
        m_aux= m.getMarker(pt,cont);
        m_aux.color.b=1.0;
        m_aux.color.a=1.0;
        m_aux.color.g=0.2;
        m_aux.color.r=0.2;
        m_aux.lifetime = ros::Duration(2);
        ret.markers.push_back(m_aux);
      }
    }
  }
  return ret;
}


visualization_msgs::Marker Planner::randomTreeMarker(const std::list<NodeState> randomTree)
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "random";
  // m.action = visualization_msgs::Marker::DELETEALL;
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::POINTS;
  m.scale.x = 0.05;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  m.color.b=0.2;
  m.color.a=1.0;
  m.color.g=1.0;
  m.color.r=0.2;
  geometry_msgs::Point pt;

  int cont = 0;
   for (auto n : randomTree) {
       pt.x = n.state[0];
       pt.y = n.state[1];
       m.points.push_back(pt); 
   }

  m.lifetime = ros::Duration(2);
  return m;
}


double Planner::retCostPath(const std::list< RRTNode >& path){
  double ret = 0;
  int cont = 0;

  for (auto it = path.begin(); it != path.end(); it++, cont++) {
    if (cont > 0) {
      double cost_node = it->cost;
      if (cost_node < 0.0){
        ROS_INFO("The value of a node is negative");
      }
      ret += cost_node;
    }
  }
  return ret;
}


#endif
