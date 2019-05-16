#include "siar_planner_action_server_simulation.hpp"
#include "ros/ros.h"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "siar_planner_node_simulation");
  
  ROS_INFO("Starting SIAR planner");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string planner_type;
  
  pnh.param("planner_type", planner_type, std::string("rrt"));

  if (planner_type == "rrt") {
    ROS_INFO("STARTING RRT planner");
    SiarPlannerActionServer<RRT> spas(nh, pnh);
    ros::spin();
  } else if (planner_type == "birrt") {
    ROS_INFO("STARTING BIRRT planner");
    SiarPlannerActionServer<biRRT> spas(nh, pnh);
    ros::spin();
  } else if (planner_type == "trrt") {
    ROS_INFO("STARTING TRRT planner");
    SiarPlannerActionServer<tRRT> spas(nh, pnh);
    ros::spin();
  } else if (planner_type == "tbirrt") {
    ROS_INFO("STARTING TBIRRT planner");
    SiarPlannerActionServer<tbiRRT> spas(nh, pnh);
    ros::spin();
  }
  
  return 0;
  
}