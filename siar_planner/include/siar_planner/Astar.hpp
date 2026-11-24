#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <siar_costmap/siarcostmap.hpp>
#include <visualization_msgs/Marker.h>

#include "siar_planner/AStarNode.h"
#include "siar_planner/NodeState.hpp"

class AStar
{
public:
  AStar(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  double getPath(NodeState start, NodeState goal, std::list<AStarNode>& path);
  visualization_msgs::Marker getPathMarker(const std::list<AStarNode>& path) const;
  visualization_msgs::Marker getGraphMarker() const;
  visualization_msgs::Marker getStateMarker(const NodeState& state, int id = 0) const;

  bool isInit() const { return map_init_; }
  std::string getFrameID() const { return map_.header.frame_id; }
  double getDeltaT() const { return delta_t_; }

protected:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  int addNode(int cell_id, const NodeState& state);
  void addEdge(int from_id, int to_id, double cost);
  void clear();
  void expandNode(int base_id);
  double heuristic_cost(const AStarNode& a, const AStarNode& b) const;
  int getBestNode() const;
  int getCellID(const NodeState& state) const;
  bool isValidCell(int column, int row) const;
  bool isNegativeCell(int cell_id) const;
  int nearestCenterlineCell(int start_cell) const;
  void buildCenterline();
  bool robotCenterToCell(double center_x, double center_y, int& column, int& row) const;
  NodeState cellCenterToState(int column, int row, double yaw) const;
  void assignPathCommands(std::list<AStarNode>& path, const NodeState& start,
                          const NodeState& goal) const;
  static double normalizeAngle(double angle);

  std::unordered_map<int, AStarNode> nodes_;
  std::unordered_set<int> closed_set_;
  std::unordered_set<int> open_set_;

  nav_msgs::OccupancyGrid map_;
  std::vector<std::uint8_t> centerline_;
  ros::Subscriber map_sub_;
  bool map_init_;
  int max_expansions_;
  double delta_t_;
};

AStar::AStar(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : map_init_(false), max_expansions_(100000), delta_t_(0.5)
{
  pnh.param("astar_max_expansions", max_expansions_, max_expansions_);
  pnh.param("delta_t", delta_t_, delta_t_);
  if (max_expansions_ <= 0) {
    ROS_WARN("AStar: astar_max_expansions must be positive; using 100000");
    max_expansions_ = 100000;
  }
  if (delta_t_ <= 0.0) {
    ROS_WARN("AStar: delta_t must be positive; using 0.5 seconds");
    delta_t_ = 0.5;
  }
  map_sub_ = nh.subscribe("/altitude_map", 2, &AStar::mapCallback, this);
}

void AStar::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map_ = *msg;
  map_init_ = true;
}

void AStar::clear()
{
  nodes_.clear();
  open_set_.clear();
  closed_set_.clear();
}

int AStar::addNode(int id, const NodeState& state)
{
  const int width = static_cast<int>(map_.info.width);
  if (width <= 0 || id < 0 || !isValidCell(id % width, id / width)) {
    return -1;
  }

  if (nodes_.find(id) == nodes_.end()) {
    AStarNode node;
    node.id = id;
    node.st = state;
    node.command_lin = 0.0;
    node.command_ang = 0.0;
    node.gScore = std::numeric_limits<double>::infinity();
    node.fScore = std::numeric_limits<double>::infinity();
    node.c_plus = 0.0;
    node.c_minus = 0.0;
    node.parent = NULL;
    nodes_.emplace(id, node);
  }
  return id;
}

void AStar::addEdge(int from_id, int to_id, double cost)
{
  nodes_.at(from_id).neighbors[to_id] = cost;
}

double AStar::getPath(NodeState start, NodeState goal, std::list<AStarNode>& path)
{
  clear();
  path.clear();
  if (!map_init_) {
    ROS_ERROR("AStar::getPath: the siar_costmap has not been initialized");
    return -1.0;
  }
  if (start.state.size() < 3 || goal.state.size() < 3) {
    ROS_ERROR("AStar::getPath: start and goal must include x, y, and yaw");
    return -2.0;
  }

  buildCenterline();
  const int start_cell = getCellID(start);
  const int goal_cell = getCellID(goal);
  if (!isNegativeCell(start_cell) || !isNegativeCell(goal_cell)) {
    ROS_ERROR("AStar::getPath: the robot center at start and goal must lie in the gutter");
    return -2.0;
  }

  const int start_line_cell = nearestCenterlineCell(start_cell);
  const int goal_line_cell = nearestCenterlineCell(goal_cell);
  if (start_line_cell < 0 || goal_line_cell < 0) {
    ROS_ERROR("AStar::getPath: could not find a centerline in the start or goal gutter");
    return -2.0;
  }

  const int width = static_cast<int>(map_.info.width);
  NodeState start_line = cellCenterToState(
    start_line_cell % width, start_line_cell / width, start.state[2]);
  NodeState goal_line = cellCenterToState(
    goal_line_cell % width, goal_line_cell / width, goal.state[2]);
  const int start_line_id = addNode(start_line_cell, start_line);
  const int goal_line_id = addNode(goal_line_cell, goal_line);
  if (start_line_id < 0 || goal_line_id < 0) {
    ROS_ERROR("AStar::getPath: centerline cell is not traversable");
    return -2.0;
  }

  const int cell_count = static_cast<int>(map_.info.width * map_.info.height);
  const int start_id = cell_count;
  const int goal_id = cell_count + 1;
  AStarNode virtual_start_node;
  virtual_start_node.id = start_id;
  virtual_start_node.st = start;
  virtual_start_node.command_lin = 0.0;
  virtual_start_node.command_ang = 0.0;
  virtual_start_node.gScore = 0.0;
  virtual_start_node.fScore = 0.0;
  virtual_start_node.c_plus = 0.0;
  virtual_start_node.c_minus = 0.0;
  virtual_start_node.parent = NULL;
  nodes_.emplace(start_id, virtual_start_node);

  AStarNode virtual_goal_node;
  virtual_goal_node.id = goal_id;
  virtual_goal_node.st = goal;
  virtual_goal_node.command_lin = 0.0;
  virtual_goal_node.command_ang = 0.0;
  virtual_goal_node.gScore = std::numeric_limits<double>::infinity();
  virtual_goal_node.fScore = std::numeric_limits<double>::infinity();
  virtual_goal_node.c_plus = 0.0;
  virtual_goal_node.c_minus = 0.0;
  virtual_goal_node.parent = NULL;
  nodes_.emplace(goal_id, virtual_goal_node);

  const NodeState& start_line_state = nodes_.at(start_line_id).st;
  const NodeState& goal_line_state = nodes_.at(goal_line_id).st;
  addEdge(start_id, start_line_id, std::hypot(
    start_line_state.state[0] - start.state[0],
    start_line_state.state[1] - start.state[1]));
  addEdge(goal_line_id, goal_id, std::hypot(
    goal.state[0] - goal_line_state.state[0],
    goal.state[1] - goal_line_state.state[1]));

  AStarNode& start_search_node = nodes_.at(start_id);
  start_search_node.fScore = heuristic_cost(start_search_node, nodes_.at(goal_id));
  open_set_.insert(start_id);

  int expanded = 0;
  while (expanded < max_expansions_ && !open_set_.empty()) {
    const int current_id = getBestNode();
    if (current_id < 0) {
      break;
    }

    if (current_id == goal_id) {
      AStarNode* current = &nodes_.at(current_id);
      while (current != NULL) {
        path.push_front(*current);
        current = current->parent;
      }
      assignPathCommands(path, start, goal);
      const double path_cost = nodes_.at(goal_id).gScore;
      return path_cost > 0.0 ? path_cost : 1e-6;
    }

    open_set_.erase(current_id);
    closed_set_.insert(current_id);
    if (current_id < cell_count) {
      expandNode(current_id);
    }

    const AStarNode& current_node = nodes_.at(current_id);
    for (const auto& edge : current_node.neighbors) {
      const int neighbor_id = edge.first;
      if (closed_set_.count(neighbor_id) != 0) {
        continue;
      }

      const double tentative_g = current_node.gScore + edge.second;
      AStarNode& neighbor = nodes_.at(neighbor_id);
      if (tentative_g >= neighbor.gScore) {
        continue;
      }

      neighbor.parent = &nodes_.at(current_id);
      neighbor.gScore = tentative_g;
      neighbor.fScore = tentative_g + heuristic_cost(neighbor, nodes_.at(goal_id));
      open_set_.insert(neighbor_id);
    }
    ++expanded;
  }

  ROS_WARN("AStar::getPath: no route found in %d expansions", expanded);
  return -1.0;
}

void AStar::expandNode(int base_id)
{
  const int width = static_cast<int>(map_.info.width);
  const int row = base_id / width;
  const int column = base_id % width;
  const NodeState base_state = nodes_.at(base_id).st;

  static const int offsets[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1}, {1, -1}, {1, 0}, {1, 1}
  };

  for (const auto& offset : offsets) {
    const int next_column = column + offset[0];
    const int next_row = row + offset[1];
    if (!isValidCell(next_column, next_row)) {
      continue;
    }

    NodeState next = cellCenterToState(next_column, next_row, 0.0);
    const double dx = next.state[0] - base_state.state[0];
    const double dy = next.state[1] - base_state.state[1];
    next.state[2] = std::atan2(dy, dx);

    const int next_id = addNode(next_row * width + next_column, next);
    if (next_id < 0) {
      continue;
    }
    const double step_cost = map_.info.resolution *
      std::sqrt(static_cast<double>(offset[0] * offset[0] + offset[1] * offset[1]));
    addEdge(base_id, next_id, step_cost);
  }
}

double AStar::heuristic_cost(const AStarNode& a, const AStarNode& b) const
{
  return std::hypot(a.st.state[0] - b.st.state[0], a.st.state[1] - b.st.state[1]);
}

int AStar::getBestNode() const
{
  double best_score = std::numeric_limits<double>::infinity();
  int best_id = -1;
  for (int id : open_set_) {
    const double score = nodes_.at(id).fScore;
    if (score < best_score) {
      best_score = score;
      best_id = id;
    }
  }
  return best_id;
}

int AStar::getCellID(const NodeState& state) const
{
  if (!map_init_ || state.state.size() < 2) {
    return -1;
  }

  int column = 0;
  int row = 0;
  if (!robotCenterToCell(state.state[0], state.state[1], column, row)) {
    return -1;
  }
  return row * static_cast<int>(map_.info.width) + column;
}

bool AStar::isValidCell(int column, int row) const
{
  if (!map_init_ || column < 0 || row < 0 ||
      column >= static_cast<int>(map_.info.width) ||
      row >= static_cast<int>(map_.info.height) ||
      centerline_.size() != map_.data.size()) {
    return false;
  }

  const std::size_t index = static_cast<std::size_t>(row) * map_.info.width + column;
  return index < centerline_.size() && centerline_[index] != 0;
}

bool AStar::isNegativeCell(int cell_id) const
{
  return cell_id >= 0 && static_cast<std::size_t>(cell_id) < map_.data.size() &&
         map_.data[cell_id] == SC_NEGATIVE_OBS;
}

int AStar::nearestCenterlineCell(int start_cell) const
{
  if (!isNegativeCell(start_cell) || centerline_.size() != map_.data.size()) {
    return -1;
  }
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  if (width <= 0 || height <= 0 ||
      map_.data.size() != static_cast<std::size_t>(width) * height) {
    return -1;
  }
  if (centerline_[start_cell] != 0) {
    return start_cell;
  }

  static const int offsets[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1}, {1, -1}, {1, 0}, {1, 1}
  };
  std::vector<std::uint8_t> visited(map_.data.size(), 0);
  std::queue<int> cells;
  cells.push(start_cell);
  visited[start_cell] = 1;

  while (!cells.empty()) {
    const int cell = cells.front();
    cells.pop();
    const int row = cell / width;
    const int column = cell % width;

    for (const auto& offset : offsets) {
      const int next_column = column + offset[0];
      const int next_row = row + offset[1];
      if (next_column < 0 || next_row < 0 ||
          next_column >= width || next_row >= height) {
        continue;
      }

      const int next_cell = next_row * width + next_column;
      if (visited[next_cell] != 0 || !isNegativeCell(next_cell)) {
        continue;
      }
      if (centerline_[next_cell] != 0) {
        return next_cell;
      }
      visited[next_cell] = 1;
      cells.push(next_cell);
    }
  }
  return -1;
}

void AStar::buildCenterline()
{
  centerline_.assign(map_.data.size(), 0);
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  if (width <= 0 || height <= 0 ||
      map_.data.size() != static_cast<std::size_t>(width) * height) {
    return;
  }

  for (std::size_t i = 0; i < map_.data.size(); ++i) {
    centerline_[i] = map_.data[i] == SC_NEGATIVE_OBS;
  }

  static const int offsets[8][2] = {
    {0, -1}, {1, -1}, {1, 0}, {1, 1},
    {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}
  };
  bool changed = true;
  while (changed) {
    changed = false;
    for (int pass = 0; pass < 2; ++pass) {
      std::vector<std::size_t> remove;
      for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
          const std::size_t index = static_cast<std::size_t>(row) * width + column;
          if (centerline_[index] == 0) {
            continue;
          }

          int neighbors[8];
          int neighbor_count = 0;
          int transitions = 0;
          for (int i = 0; i < 8; ++i) {
            const int next_column = column + offsets[i][0];
            const int next_row = row + offsets[i][1];
            const bool occupied =
              next_column >= 0 && next_column < width &&
              next_row >= 0 && next_row < height &&
              centerline_[static_cast<std::size_t>(next_row) * width + next_column] != 0;
            neighbors[i] = occupied ? 1 : 0;
            neighbor_count += neighbors[i];
          }
          for (int i = 0; i < 8; ++i) {
            if (neighbors[i] == 0 && neighbors[(i + 1) % 8] == 1) {
              ++transitions;
            }
          }

          if (neighbor_count < 2 || neighbor_count > 6 || transitions != 1) {
            continue;
          }

          const bool remove_this_pass = pass == 0
            ? neighbors[0] * neighbors[2] * neighbors[4] == 0 &&
              neighbors[2] * neighbors[4] * neighbors[6] == 0
            : neighbors[0] * neighbors[2] * neighbors[6] == 0 &&
              neighbors[0] * neighbors[4] * neighbors[6] == 0;
          if (remove_this_pass) {
            remove.push_back(index);
          }
        }
      }

      if (!remove.empty()) {
        changed = true;
        for (std::size_t index : remove) {
          centerline_[index] = 0;
        }
      }
    }
  }
}

bool AStar::robotCenterToCell(
  double center_x, double center_y, int& column, int& row) const
{
  const double resolution = map_.info.resolution;
  if (resolution <= 0.0 || map_.info.width == 0 || map_.info.height == 0) {
    return false;
  }

  const geometry_msgs::Quaternion& q = map_.info.origin.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double dx = center_x - map_.info.origin.position.x;
  const double dy = center_y - map_.info.origin.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

  if (!std::isfinite(local_x) || !std::isfinite(local_y) ||
      local_x < 0.0 || local_y < 0.0 ||
      local_x >= map_.info.width * resolution ||
      local_y >= map_.info.height * resolution) {
    return false;
  }

  column = static_cast<int>(std::floor(local_x / resolution));
  row = static_cast<int>(std::floor(local_y / resolution));
  return true;
}

NodeState AStar::cellCenterToState(int column, int row, double yaw) const
{
  const double resolution = map_.info.resolution;
  const geometry_msgs::Quaternion& q = map_.info.origin.orientation;
  const double origin_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double local_x = (column + 0.5) * resolution;
  const double local_y = (row + 0.5) * resolution;

  NodeState state;
  state.state.resize(3);
  state.state[0] = map_.info.origin.position.x +
    std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y;
  state.state[1] = map_.info.origin.position.y +
    std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y;
  state.state[2] = yaw;
  return state;
}

void AStar::assignPathCommands(
  std::list<AStarNode>& path, const NodeState& start, const NodeState& goal) const
{
  if (path.empty()) {
    return;
  }

  path.front().st = start;
  path.front().command_lin = 0.0;
  path.front().command_ang = 0.0;
  if (path.size() == 1) {
    path.back().st = goal;
    path.back().command_lin =
      std::hypot(goal.state[0] - start.state[0], goal.state[1] - start.state[1]) / delta_t_;
    path.back().command_ang = normalizeAngle(goal.state[2] - start.state[2]) / delta_t_;
    return;
  }

  auto previous = path.begin();
  for (auto current = std::next(previous); current != path.end(); ++current) {
    const bool is_goal = std::next(current) == path.end();
    if (is_goal) {
      current->st = goal;
    } else {
      current->st.state[2] = std::atan2(
        current->st.state[1] - previous->st.state[1],
        current->st.state[0] - previous->st.state[0]);
    }

    const double dx = current->st.state[0] - previous->st.state[0];
    const double dy = current->st.state[1] - previous->st.state[1];
    current->command_lin = std::hypot(dx, dy) / delta_t_;
    current->command_ang =
      normalizeAngle(current->st.state[2] - previous->st.state[2]) / delta_t_;
    previous = current;
  }
}

double AStar::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

visualization_msgs::Marker AStar::getPathMarker(const std::list<AStarNode>& path) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = getFrameID();
  marker.header.stamp = ros::Time::now();
  marker.ns = "astar_path";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = std::max(map_.info.resolution * 0.4, 0.01);
  marker.color.r = 1.0;
  marker.color.g = 0.2;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (const AStarNode& node : path) {
    geometry_msgs::Point point;
    point.x = node.st.state[0];
    point.y = node.st.state[1];
    marker.points.push_back(point);
  }
  return marker;
}

visualization_msgs::Marker AStar::getGraphMarker() const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = getFrameID();
  marker.header.stamp = ros::Time::now();
  marker.ns = "astar_graph";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = std::max(map_.info.resolution * 0.2, 0.005);
  marker.color.r = 0.2;
  marker.color.g = 0.4;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  for (const auto& entry : nodes_) {
    const AStarNode& node = entry.second;
    for (const auto& edge : node.neighbors) {
      if (node.id >= edge.first) {
        continue;
      }
      const AStarNode& neighbor = nodes_.at(edge.first);
      geometry_msgs::Point from;
      geometry_msgs::Point to;
      from.x = node.st.state[0];
      from.y = node.st.state[1];
      to.x = neighbor.st.state[0];
      to.y = neighbor.st.state[1];
      marker.points.push_back(from);
      marker.points.push_back(to);
    }
  }
  return marker;
}

visualization_msgs::Marker AStar::getStateMarker(const NodeState& state, int id) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = getFrameID();
  marker.header.stamp = ros::Time::now();
  marker.ns = "astar_states";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = state.state[0];
  marker.pose.position.y = state.state[1];
  marker.pose.orientation.z = std::sin(state.state[2] * 0.5);
  marker.pose.orientation.w = std::cos(state.state[2] * 0.5);
  const double marker_size = std::max(static_cast<double>(map_.info.resolution), 0.05);
  marker.scale.x = marker.scale.y = marker.scale.z = marker_size;
  marker.color.r = id == 0 ? 0.0 : 1.0;
  marker.color.g = id == 0 ? 1.0 : 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  return marker;
}

#endif
