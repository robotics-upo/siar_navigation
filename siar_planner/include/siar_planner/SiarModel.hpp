#ifndef SIAR_MODEL__HPP__
#define SIAR_MODEL__HPP__

#include "siar_controller/command_evaluator.hpp"
#include "siar_planner/NodeState.hpp"
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <random>

class SiarModel 
{
public:
  //! @brief Constructor from NodeHandles
  SiarModel(ros::NodeHandle &nh, ros::NodeHandle &pn);
  
  void occupancyGridCallback(nav_msgs::OccupancyGridConstPtr msg);
  
  visualization_msgs::Marker testIntegration(NodeState &st, bool relaxed = false);
  
  //! @brief Integrates the model and returns the cost associated with 
  //! @return Negative --> collision. Positive --> Arc's longitude
  double integrate(NodeState &st, geometry_msgs::Twist &cmd, double T, bool relaxed = false);
  double integrateTransition(NodeState &st, geometry_msgs::Twist &cmd, double T);
  double integrateTransition2(NodeState &st, geometry_msgs::Twist &cmd, double T);
  
  //! @brief Integrates the model and returns the cost associated with 
  //! @return Negative --> collision. Positive --> Arc's longitude
  double integrate(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T, bool relaxed);
  double integrateTransition(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T);
  double integrateTransition2(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T);
  
  virtual geometry_msgs::Twist generateRandomCommand();
  
  inline bool isInit() const {return map_init;}

  // inline double costWheelsQnew() const {return cost_wheels;}
  double costWheelsQnear(double x, double y, double th);

  
  inline std::string getFrameID() const {
    if (map_init) {
      return m_world.header.frame_id;
    }
  }
  
  visualization_msgs::Marker getMarker(NodeState &st, int id = 0); 
  
  inline bool isCollision(NodeState &st) {
    bool ret_val;
    m_ce.applyFootprint(st.state[0], st.state[1], st.state[2], m_world, ret_val);
    return ret_val;
  }
  
  inline double getMinWheel() const {
//     return m_ce.getMinWheel();
    return -1.0;
  }
  
  inline void decreaseWheels(double decrement, double last_wheel) {
    double m_w = m_ce.getMinWheelLeft() - decrement; // Decrease the minimum allowed wheel on the floor 
    m_w = (m_w < last_wheel)?last_wheel:m_w; // Check if the allowed wheel is below the maximum
    m_ce.setMinWheelLeft(m_w);
    // Right wheel
    m_w = m_ce.getMinWheelRight() - decrement; // Decrease the minimum allowed wheel on the floor 
    m_w = (m_w < last_wheel)?last_wheel:m_w; // Check if the allowed wheel is below the maximum
    m_ce.setMinWheelRight(m_w); 
  }
  
  inline void setMinWheel(double v) {
    
    m_ce.setMinWheelLeft(v);
    m_ce.setMinWheelRight(v);
  }
  
  inline double getWorldMaxX() const{
    return m_world.info.resolution * m_world.info.height * 0.5;
  }
  inline double getWorldMaxY() const{
    return m_world.info.resolution * m_world.info.width * 0.5;
  }
  
protected:
  nav_msgs::OccupancyGrid m_world;
  siar_controller::CommandEvaluator m_ce;
  double cost_wheels, cost_wheels_q_near;
  
  bool map_init;
  
  template< class RealType = double > class uniform_real_distribution;
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
  
  ros::Subscriber map_sub;
  
  visualization_msgs::Marker m;
};

SiarModel::SiarModel(ros::NodeHandle &nh, ros::NodeHandle& pn):m_ce(pn), map_init(false), gen(rd()), dis(-m_ce.getCharacteristics().theta_dot_max, m_ce.getCharacteristics().theta_dot_max)
{
  map_sub = nh.subscribe("/altitude_map", 2, &SiarModel::occupancyGridCallback, this);
}

void SiarModel::occupancyGridCallback(nav_msgs::OccupancyGridConstPtr msg)
{
  map_init = true;
  m_world = *msg;
  m_ce.initializeFootprint(*msg);
  
}

double SiarModel::integrate(NodeState& st, geometry_msgs::Twist& cmd, double T, bool relaxed)
{
  return integrate(m, st, cmd, T, relaxed);
}


double SiarModel::integrate(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T, bool relaxed)
{
  geometry_msgs::Twist v_ini;
  v_ini.linear.x = m_ce.getCharacteristics().v_max;
  
  if (st.state.size() < 3) {
    ROS_ERROR("SiarModel::integrate --> cannot integrate the model --> too few states. State size: %u", (unsigned int) st.state.size());
  }
  
  // Debug:
//   ROS_INFO("Calling evaluate trajectory. Delta_t = %f. T_hor = %f. ", m_ce.
  
  m_ce.setDeltaT(T);
  
  double ret_val;
  if (!relaxed) 
    ret_val = m_ce.evaluateTrajectory(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2]);
  else
    ret_val = m_ce.evaluateTrajectoryRelaxed(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2]);
  
  st.state = m_ce.getLastState();
  
  
  return ret_val;
}

double SiarModel::integrateTransition(NodeState& st, geometry_msgs::Twist& cmd, double T)
{
  return integrateTransition(m, st, cmd, T);
}

double SiarModel::integrateTransition(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T)
{
  geometry_msgs::Twist v_ini;
  v_ini.linear.x = m_ce.getCharacteristics().v_max;
  
  if (st.state.size() < 3) {
    ROS_ERROR("SiarModel::integrate --> cannot integrate the model --> too few states. State size: %u", (unsigned int) st.state.size());
  }
  
  m_ce.setDeltaT(T);
  double ret_val;
  ret_val = m_ce.evaluateTrajectoryTransition(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2]); 
  st.state = m_ce.getLastState();
  cost_wheels = m_ce.getCostWheelsQnew();
  
  return ret_val;
}

double SiarModel::integrateTransition2(NodeState& st, geometry_msgs::Twist& cmd, double T)
{
  return integrateTransition2(m, st, cmd, T);
}

double SiarModel::integrateTransition2(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T)
{
  geometry_msgs::Twist v_ini;
  v_ini.linear.x = m_ce.getCharacteristics().v_max;
  
  if (st.state.size() < 3) {
    ROS_ERROR("SiarModel::integrate --> cannot integrate the model --> too few states. State size: %u", (unsigned int) st.state.size());
  }
  
  m_ce.setDeltaT(T);
  double ret_val;
  ret_val = m_ce.evaluateTrajectoryTransition(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2]); 
  st.state = m_ce.getLastState();
  cost_wheels = m_ce.getCostWheelsQnew();
  
  return ret_val;
}

double SiarModel::costWheelsQnear(double x, double y, double th){

  bool collision = false;
  bool collision_wheels=false;

  double cost_wheels_q_near = m_ce.applyFootprintTransition(x,  y, th, m_world, collision, collision_wheels); 

  return cost_wheels_q_near;
}

// double SiarModel::integrate(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T, bool relaxed)
// {
//   geometry_msgs::Twist v_ini;
//   double ret_val;

//   if (st.state.size() < 3) {
//     ROS_ERROR("SiarModel::integrate --> cannot integrate the model --> too few states. State size: %u", (unsigned int) st.state.size());
//   }

//   v_ini.linear.x = m_ce.getCharacteristics().v_max;
//   m_ce.setDeltaT(T);
//   ret_val = m_ce.evaluateTrajectoryTransition(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2]);
//   st.state = m_ce.getLastState();
    
//   return ret_val;
// }


geometry_msgs::Twist SiarModel::generateRandomCommand() {
  
  geometry_msgs::Twist ret;
  
  ret.linear.x = m_ce.getCharacteristics().v_max;
  ret.angular.z = dis(gen);
  ret.linear.y = ret.linear.z = ret.angular.x = ret.angular.y = 0.0;
  
  return ret;
}

visualization_msgs::Marker SiarModel::getMarker(NodeState& st, int id)
{
  visualization_msgs::Marker m;
  
  if (m_ce.getFootprint() != NULL) {
    m_ce.getFootprint()->addPoints(st.state[0], st.state[1], st.state[2], m, id, true, m_world.header.frame_id);
  } else {
    ROS_ERROR("SiarModel::getMarker: --> Footprint is not initialized");
  }
  
  return m;
}

visualization_msgs::Marker SiarModel::testIntegration(NodeState& st, bool relaxed)
{
  auto comm = generateRandomCommand();
  double cost = integrate(st, comm, 0.5, relaxed);
  
  ROS_INFO("SiarModel::testIntegration --> Command = %f, %f --> Cost = %f", comm.linear.x, comm.angular.z, cost);
  
  
  return m;
}



#endif