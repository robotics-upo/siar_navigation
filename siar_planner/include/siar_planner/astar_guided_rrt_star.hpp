#ifndef ASTAR_GUIDED_RRT_STAR_HPP_
#define ASTAR_GUIDED_RRT_STAR_HPP_

#include "siar_planner/Astar.hpp"
#include "siar_planner/rrt_star.hpp"

#include <algorithm>
#include <vector>

class AStarGuidedRRTStar : public RRTStar
{
public:
  AStarGuidedRRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path) override;

protected:
  NodeState getSampleState(int iteration) override;

private:
  AStar a_star_;
  std::vector<NodeState> guided_states_;
  double waypoint_bias_;
  int waypoint_count_;
};

AStarGuidedRRTStar::AStarGuidedRRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : RRTStar(nh, pnh), a_star_(nh, pnh), waypoint_bias_(0.2), waypoint_count_(10)
{
  pnh.param("astar_waypoint_bias", waypoint_bias_, waypoint_bias_);
  pnh.param("astar_waypoint_count", waypoint_count_, waypoint_count_);

  if (waypoint_bias_ < 0.0 || waypoint_bias_ > 1.0) {
    ROS_WARN("astar_waypoint_bias must be in [0, 1]; clamping its value");
    waypoint_bias_ = std::max(0.0, std::min(1.0, waypoint_bias_));
  }
  if (waypoint_count_ < 1) {
    ROS_WARN("astar_waypoint_count must be positive; using 1");
    waypoint_count_ = 1;
  }
}

double AStarGuidedRRTStar::resolve(
  NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  path.clear();
  guided_states_.clear();
  clear();

  if (!a_star_.isInit()) {
    ROS_ERROR("AStarGuidedRRTStar::resolve: A* map model is not initialized");
    return -2.0;
  }

  std::list<AStarNode> astar_path;
  const double astar_cost = a_star_.getPath(start, goal, astar_path);
  if (astar_cost < 0.0 || astar_path.empty()) {
    ROS_WARN("AStarGuidedRRTStar::resolve: A* could not find a route");
    return -4.0;
  }

  std::vector<NodeState> states;
  states.reserve(astar_path.size());
  for (const AStarNode& node : astar_path) {
    states.push_back(node.st);
  }

  const std::size_t sample_count = std::min(
    states.size(), static_cast<std::size_t>(waypoint_count_));
  guided_states_.reserve(sample_count);
  if (sample_count == 1) {
    guided_states_.push_back(states.back());
  } else {
    for (std::size_t i = 0; i < sample_count; ++i) {
      const std::size_t index = i * (states.size() - 1) / (sample_count - 1);
      guided_states_.push_back(states[index]);
    }
  }

  ROS_INFO(
    "AStarGuidedRRTStar: A* route has %u states; using %u guide samples",
    static_cast<unsigned int>(states.size()),
    static_cast<unsigned int>(guided_states_.size()));

  const double result = RRTStar::resolve(start, goal, path);
  guided_states_.clear();
  return result;
}

NodeState AStarGuidedRRTStar::getSampleState(int iteration)
{
  if (samp_goal_rate > 0 && iteration % samp_goal_rate == 0) {
    return goal_node.st;
  }

  if (!guided_states_.empty() && dis(gen) < waypoint_bias_) {
    const std::size_t index = static_cast<std::size_t>(
      dis(gen) * guided_states_.size());
    return guided_states_[std::min(index, guided_states_.size() - 1)];
  }

  return RRTStar::getSampleState(iteration);
}

#endif
