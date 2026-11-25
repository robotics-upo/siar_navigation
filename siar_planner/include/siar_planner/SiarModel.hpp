#ifndef SIAR_MODEL__HPP__
#define SIAR_MODEL__HPP__

#include "siar_controller/command_evaluator.hpp"
#include "siar_planner/NodeState.hpp"
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

class SiarModel 
{
public:
  //! @brief Constructor from NodeHandles
  SiarModel(ros::NodeHandle &nh, ros::NodeHandle &pn);
  
  void occupancyGridCallback(nav_msgs::OccupancyGridConstPtr msg);
  
  visualization_msgs::Marker testIntegration(NodeState &st);
  
  //! @brief Integrates the model and returns the cost associated with 
  //! @return Negative --> collision. Positive --> Arc's longitude
  double integrate(NodeState &st, geometry_msgs::Twist &cmd, double T);
  
  //! @brief Integrates the model and returns the cost associated with 
  //! @return Negative --> collision. Positive --> Arc's longitude
  double integrate(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T);
  
  virtual geometry_msgs::Twist generateRandomCommand();
  virtual geometry_msgs::Twist generateRandomCommandJustRot();
  
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
  }

  inline double getWidth(double command) {
    if (command < 0.0) 
      return -1.0;
    if (command < 10) {
      return 0.51 + 0.007 * command;
    } else if (command < 20) {
      return 0.58 + 0.006 * (command - 10);
    } else if (command < 30) {
      return 0.64 + 0.004 * (command - 20);
    } else if (command < 40) {
      return 0.68 + 0.002 * (command - 30);
    } else if (command < 50) {
      return 0.7 + 0.001 * (command - 40);
    } else if (command < 60) {
      return 0.71;
    } else if (command < 70) {
      return 0.71;
    } else if (command < 80) {
      return 0.71 - 0.001 * (command - 70);
    } else if (command < 90) {
      return 0.7 - 0.0001 * (command - 80);
    } else if (command < 100) {
      return 0.69 - 0.0001 * (command - 90); 
    } else if (command < 110) {
      return 0.68 + 0.0002 * (command - 100);
    } else if (command < 120) {
      return 0.66 - 0.0002 * (command - 110);
    } else if (command < 130) {
      return 0.64 - 0.0003 * (command - 120);
    } else if (command < 140) {
      return 0.61 - 0.0003 * (command - 130);
    } else if (command < 150) {
      return 0.58 - 0.0006 * (command - 140);
    }
    return 0.51;
  }

  inline double getXElec(double command) {
    if (command < 0.0) 
      return -1.0;
    if (command < 10) {
      return 0.14 - 0.002 * command;
    } else if (command < 20) {
      return 0.12 - 0.003 * (command - 10);
    } else if (command < 30) {
      return 0.09 - 0.003 * (command - 20);
    } else if (command < 40) {
      return 0.06 - 0.002 * (command - 30);
    } else if (command < 50) {
      return 0.04 - 0.002 * (command - 40);
    } else if (command < 60) {
      return 0.02 - 0.002 * (command - 50);
    } else if (command < 70) {
      return 0.00 - 0.001 * (command - 60);
    } else if (command < 80) {
      return -0.01 - 0.002 * (command - 70);
    } else if (command < 90) {
      return -0.03 - 0.002 * (command - 80);
    } else if (command < 100) {
      return -0.05 - 0.001 * (command - 90);
    } else if (command < 110) {
      return -0.07 - 0.002 * (command - 100);
    } else if (command < 120) {
      return -0.08 - 0.0002 * (command - 110);
    } else if (command < 130) {
      return -0.10 - 0.001 * (command - 120);
    } else if (command < 140) {
      return -0.11 - 0.001 * (command - 130);
    } else if (command < 150) {
      return -0.12 - 0.002 * (command - 140);
    }
    return -0.14;
  }

  double getWidthMotorCommand(const NodeState& state) const {
    return state.state.size() > 3 ? state.state[3] : 60.0;
  }

  bool setWidthMotorRef(double command) {
    if (!std::isfinite(command) || command < 0.0 || command > 150.0) {
      ROS_ERROR("SiarModel::setWidthMotorRef: invalid width command %.3f", command);
      return false;
    }
    return m_ce.setWidthElec(getWidth(command), getXElec(command));
  }

  bool changeWidthCommand(
    NodeState& state, bool increase, double minimum_width, double maximum_width)
  {
    if (!map_init || state.state.size() < 3 ||
        !std::isfinite(minimum_width) || !std::isfinite(maximum_width) ||
        minimum_width > maximum_width) {
      return false;
    }

    const double current_command = getWidthMotorCommand(state);
    if (!std::isfinite(current_command) ||
        current_command < 0.0 || current_command > 150.0) {
      return false;
    }

    const double current_width = getWidth(current_command);
    const double direction = increase ? 1.0 : -1.0;
    const double desired_width = std::max(
      minimum_width,
      std::min(maximum_width, current_width + direction * width_change_step_m_));
    if ((increase && desired_width <= current_width) ||
        (!increase && desired_width >= current_width)) {
      return false;
    }

    int best_command = -1;
    double best_error = std::numeric_limits<double>::infinity();
    for (int command = 0; command <= 150; ++command) {
      const double candidate_width = getWidth(command);
      if (candidate_width < minimum_width || candidate_width > maximum_width ||
          (increase && candidate_width <= current_width) ||
          (!increase && candidate_width >= current_width)) {
        continue;
      }

      const double error = std::fabs(candidate_width - desired_width);
      if (error < best_error) {
        best_error = error;
        best_command = command;
      }
    }
    if (best_command < 0) {
      return false;
    }

    NodeState candidate = state;
    if (candidate.state.size() < 4) {
      candidate.state.resize(4);
      candidate.state[3] = current_command;
    }
    candidate.state[3] = best_command;
    if (!setWidthMotorRef(best_command)) {
      return false;
    }
    if (isCollision(candidate)) {
      setWidthMotorRef(current_command);
      return false;
    }

    state = candidate;
    return true;
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
  
  siar_controller::CommandEvaluator m_ce;
  nav_msgs::OccupancyGrid m_world;


protected:
  // nav_msgs::OccupancyGrid m_world;
  // siar_controller::CommandEvaluator m_ce;
  double cost_wheels, cost_wheels_q_near;
  double width_change_step_m_;
  bool map_init;
  
  template< class RealType = double > class uniform_real_distribution;
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
  
  ros::Subscriber map_sub;
  
  visualization_msgs::Marker m;
};

SiarModel::SiarModel(ros::NodeHandle &nh, ros::NodeHandle& pn)
  : m_ce(pn), width_change_step_m_(0.02), map_init(false), gen(rd()),
    dis(-m_ce.getCharacteristics().theta_dot_max, m_ce.getCharacteristics().theta_dot_max)
{
  pn.param("width_change_step_m", width_change_step_m_, width_change_step_m_);
  if (!std::isfinite(width_change_step_m_) || width_change_step_m_ <= 0.0) {
    ROS_WARN("width_change_step_m must be positive; using 0.02 m");
    width_change_step_m_ = 0.02;
  }
  map_sub = nh.subscribe("/altitude_map", 2, &SiarModel::occupancyGridCallback, this);
}

void SiarModel::occupancyGridCallback(nav_msgs::OccupancyGridConstPtr msg)
{
  map_init = true;
  m_world = *msg;
  m_ce.initializeFootprint(*msg);
  
}

double SiarModel::integrate(NodeState& st, geometry_msgs::Twist& cmd, double T)
{
  return integrate(m, st, cmd, T);
}

double SiarModel::integrate(visualization_msgs::Marker& m, NodeState& st, geometry_msgs::Twist& cmd, double T)
{
  geometry_msgs::Twist v_ini;
  v_ini.linear.x = m_ce.getCharacteristics().v_max;


  const double command_mot = getWidthMotorCommand(st);
  if (!setWidthMotorRef(command_mot)) {
    return -1.0;
  }
  
  
  if (st.state.size() < 3) {
    ROS_ERROR("SiarModel::integrate --> cannot integrate the model --> too few states. State size: %u", (unsigned int) st.state.size());
  }
  
  m_ce.setDeltaT(T);
  
  double ret_val;

  ret_val = m_ce.evaluateTrajectory(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2]);

  st.state = m_ce.getLastState();
  
  if (st.state.size() < 4) {
    st.state.push_back(command_mot);
  }
  
  return ret_val;
}

double SiarModel::costWheelsQnear(double x, double y, double th){
  return m_ce.getWheelObstacleCost(x, y, th, m_world);
}

geometry_msgs::Twist SiarModel::generateRandomCommand() {
  
  geometry_msgs::Twist ret;
  
  ret.linear.x = m_ce.getCharacteristics().v_max;
  ret.angular.z = dis(gen);
  ret.linear.y = ret.linear.z = ret.angular.x = ret.angular.y = 0.0;
  
  return ret;
}

geometry_msgs::Twist SiarModel::generateRandomCommandJustRot() {
  
  geometry_msgs::Twist ret;

  ret.linear.x = 0.02;
  ret.angular.z = (dis(gen));
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

visualization_msgs::Marker SiarModel::testIntegration(NodeState& st)
{
  auto comm = generateRandomCommand();
  double cost = integrate(st, comm, 0.5);
  
  ROS_INFO("SiarModel::testIntegration --> Command = %f, %f --> Cost = %f", comm.linear.x, comm.angular.z, cost);
  
  return m;
}

#endif