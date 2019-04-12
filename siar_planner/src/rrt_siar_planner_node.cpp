#include "rrt_siar_planner_action_server.hpp"
#include "ros/ros.h"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rrt_siar_planner_node");
  
  ROS_INFO("Starting SIAR planner");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string planner_type;
  
  pnh.param("planner_type", planner_type, std::string("rrt"));

  if (planner_type == "rrt") {
    SiarPlannerActionServer<RRT> spas(nh, pnh);
    ros::spin();
  } else if (planner_type == "birrt") {
    SiarPlannerActionServer<biRRT> spas(nh, pnh);
    ros::spin();
  } else if (planner_type == "trrt") {
    SiarPlannerActionServer<tRRT> spas(nh, pnh);
    ros::spin();
  } else if (planner_type == "tbirrt") {
    SiarPlannerActionServer<tbiRRT> spas(nh, pnh);
    ros::spin();
  }
  
  return 0;
  
}