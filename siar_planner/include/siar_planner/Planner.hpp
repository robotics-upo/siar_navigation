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
//   RRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
//   ~RRT();
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path) = 0;      
  virtual double retCostPath(const std::list< RRTNode >& path);

  SiarModel &getModel() {return m;}

  virtual visualization_msgs::MarkerArray getPathMarker(const std::list< RRTNode >& path);
  
  virtual visualization_msgs::Marker getGraphMarker() = 0;
  
  double getDeltaT() const {return delta_t;}

  virtual int getGraphSize() = 0;

  RRTNode goal_node, start_node;
  
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
  double max_x, max_y, max_yaw, min_x, min_y, min_yaw; //de que tipo serian?
  
  SiarModel m;
  double goal_gap_m, goal_gap_rad;
  
  int samp_cont, samp_goal_rate; //contadores de sampleo
  
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
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
    goal_gap_m = 0; //que pongo aqui
  }
  
  if (!pnh.getParam("goal_gap_rad", goal_gap_rad)) {
    goal_gap_rad = 0; //que pongo aqui
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
  
  if (!pnh.getParam("min_x", min_x)) { //ver estos valores
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
  
  if (!pnh.getParam("cellsize_m", cellsize_m)) { //si quito esto, debo cambiar getGraphMarker
    cellsize_m = 0.2;
  }
  if (!pnh.getParam("cellsize_rad", cellsize_rad)) {
    cellsize_rad = 0.2;
  }
  
  ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K); //cambiar esto
}

NodeState Planner::getRandomState(double max_x, double min_x, double max_y, double min_y, double max_yaw, double min_yaw) {
  NodeState randomState;
  randomState.state.resize(3);
  randomState.state[0] = (dis(gen) * (max_x-min_x)) + min_x;  //set random value for each state varible
  randomState.state[1] = (dis(gen) * (max_y-min_y)) + min_y;
  randomState.state[2] = (dis(gen) * (max_yaw-min_yaw)) + min_yaw;
  
  return randomState;
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
      geometry_msgs::Twist command;
      command.linear.x = it->command_lin;
      command.angular.z = it->command_ang;
      //  m_aux.action = visualization_msgs::Marker::DELETEALL;
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
