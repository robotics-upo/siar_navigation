#ifndef SIAR_PLANNER_ACTION_SERVER_HPP__
#define SIAR_PLANNER_ACTION_SERVER_HPP__

#include <ros/ros.h>

#include "siar_planner/rrt.hpp"
#include "siar_planner/biRRT.hpp"
#include "siar_planner/trrt.hpp"
#include "siar_planner/tbiRRT.hpp"

#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//! @class This class takes as inputs a desired 2D pose (with yaw) or a desired direction in the fork and tries to get a path

template <typename T> class SiarPlannerActionServer 
{
public:
  
  SiarPlannerActionServer(ros::NodeHandle& nh, ros::NodeHandle& pnh):seq(0),reverse(false)
{
  // Publishers
  path_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("path_marker", 2, true);
  graph_pub = nh.advertise<visualization_msgs::Marker>("graph_marker", 2, true);
  path_pub = nh.advertise<nav_msgs::Path>("planned_path", 2, true);

  // Subscribers
  reverse_sub = nh.subscribe("/reverse", 1, &SiarPlannerActionServer::reverseCB, this);
  goal_sub = nh.subscribe("/move_base_simple/goal", 1, &SiarPlannerActionServer::goalCB, this);

  planner = new T(nh,pnh);
  pnh.param("base_frame_id", base_frame_id, std::string("base_link"));
  pnh.param("global_frame_id", base_frame_id, std::string("world"));
}

  double getDeltaT() const {return planner->getDeltaT();}
  
protected:
  ros::NodeHandle nh_;
  ros::Subscriber reverse_sub, goal_sub; // Also the commands can be sent as a regular topic (without action server)
  ros::Publisher path_marker_pub, graph_pub, path_pub;
  tf::TransformListener tfl;

  std::string base_frame_id, global_frame_id; // Frame of the robot (usually /base_link)
  int seq;
  
  // Path related info
  std::list<RRTNode> curr_path;
  
  // Choose a Planner
  T *planner;
  
  // Status flag
  bool reverse;
  
  void goalCB(const geometry_msgs::PoseStamped_< std::allocator< void > >::ConstPtr& msg ) {
    calculatePath(*msg);
  }
  
  // ROS Communication stuff -----------------------------
  //! @brief To cancel an ongoing action
  void preemptCB();
  
  void reverseCB(const std_msgs::Bool::ConstPtr &msg) {
    reverse = (msg->data != 0);
  }
  
  //! @brief Calculates path and gets the solution
  void calculatePath(const geometry_msgs::PoseStamped &pose) {
    NodeState start, goal;
    ros::Time t0, t1;
    t0 = ros::Time::now();
    start.state.push_back(0);start.state.push_back(0);start.state.push_back(0);
    
    // TODO: transform the pose (now it assumes it is indicated in base_link)
    geometry_msgs::PoseStamped tf_pose = pose;
    geometry_msgs::PoseStamped pose_cpy = pose;
    try {
      // tfl.waitForTransform(base_frame_id, pose.header.frame_id, ros::Time(0));
      pose_cpy.header.stamp = ros::Time(0);



      if (pose.header.frame_id != base_frame_id) {
        tfl.transformPose(base_frame_id, pose_cpy, tf_pose);
      }
    } catch (std::exception &e) {
      ROS_ERROR("siar_planner_as::calculatePath --> could not transform the pose. Base frame = %s. Pose frame = %s", base_frame_id.c_str(), pose.header.frame_id.c_str());
      return;
    }
    
    goal.state.push_back(tf_pose.pose.position.x);
    goal.state.push_back(tf_pose.pose.position.y);
    
    // Conversion from quaternion message to yaw
    double roll, pitch, yaw;
    tf::Quaternion q(tf_pose.pose.orientation.x, tf_pose.pose.orientation.y, tf_pose.pose.orientation.z, tf_pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    goal.state.push_back(yaw);
    
    
    double cost = planner->resolve(start, goal, curr_path);
    
    t1 = ros::Time::now();
    ROS_INFO("Path calculated. Cost = %f.\t Expended time: %f", cost, (t1 - t0).toSec());
    
    visualization_msgs::MarkerArray m_a = planner->getPathMarker(curr_path);

    // Transform the path to a global frame
    tf::StampedTransform t;
    t.setIdentity();
    try { 
      if (m_a.markers.size() > 0 && m_a.markers.at(0).header.frame_id != global_frame_id) {
        tfl.lookupTransform(global_frame_id, m_a.markers[0].header.frame_id, ros::Time(0), t);
      }
      auto ang = t.getRotation().getAngle();
      auto axis = t.getRotation().getAxis();
      for (auto m:m_a.markers) {
        m.header.frame_id = global_frame_id;
        for (auto p_:m.points) {
          tf::Point p;
          tf::pointMsgToTF(p_, p);
          p.rotate(axis, ang);
          p = p + t.getOrigin();
          tf::pointTFToMsg(p, p_);
        }
      }
    } catch (std::exception &e) {
      ROS_ERROR("siar_planner_as::calculatePath --> could not transform the pose");
    }
    path_marker_pub.publish(m_a);
    graph_pub.publish(planner->getGraphMarker());

    nav_msgs::Path path;
    path.header.frame_id = base_frame_id;
    path.header.stamp = ros::Time::now();

    auto time_ = ros::Time::now();
    for (auto x:curr_path) {
      geometry_msgs::PoseStamped p;
      p.header.stamp = time_;
      p.header.frame_id = base_frame_id;
      p.header.seq = seq++;
      // time_ = time_ + ros::Duration(planner->getDeltaT);
      p.pose.position.x = x.st.state[0];
      p.pose.position.y = x.st.state[1];
      tf::Quaternion q;
      q.setRPY(0,0, x.st.state[2]);
      tf::quaternionTFToMsg(q, p.pose.orientation);
      path.poses.push_back(p);
    }

    path_pub.publish(path); 
  }
};

#endif