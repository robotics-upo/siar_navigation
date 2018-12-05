#include "siar_planner/rrt.hpp"
#include "siar_planner/biRRT.hpp"
#include "ros/ros.h"

using functions::RealVector;

int main(int argc, char** argv){
  
  ros::init(argc, argv, "test_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
//   RRT a(nh, pnh);
  biRRT a(nh, pnh);
  
  ROS_INFO("Waiting for map initialization");
  while (!a.getModel().isInit() && ros::ok()){ 
    ros::spinOnce();
    sleep(1);
  }
  ROS_INFO("Map initialized");
  
  // Get parameters from ROS
  double x_0, y_0, a_0;
  pnh.param("x0", x_0, 0.0);
  pnh.param("y0", y_0, 0.0);
  pnh.param("a0", a_0, 0.0);
  
  double x_g, y_g, a_g;
  pnh.param("x_g", x_g, 0.0);
  pnh.param("y_g", y_g, 0.0);
  pnh.param("a_g", a_g, 0.0);
  
  ROS_INFO("START: (%f, %f, %f)\t\tGOAL: (%f, %f, %f)", x_0, y_0, a_0, x_g, y_g, a_g);
  
  NodeState init, goal; 
  RealVector v;
  v.push_back(x_0);v.push_back(y_0);v.push_back(a_0);
  init.state = v;
  v.clear();
  v.push_back(x_g);v.push_back(y_g);v.push_back(a_g);
  goal.state = v;
  
  std::list<RRTNode> path; 
  
  
  
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("init_marker", 2, true);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("goal_marker", 2, true);
  ros::Publisher graph_pub = nh.advertise<visualization_msgs::Marker>("graph_marker", 2, true);
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 2, true);
  
  ROS_INFO("Publishing visualization markers (init and goal)");
  vis_pub.publish(a.getModel().getMarker(init));
  goal_pub.publish(a.getModel().getMarker(goal, 1));
  
  ROS_INFO("Calling to resolve");
  ros::Time t = ros::Time::now();
  double cost = a.resolve(init, goal, path);
  std::cout << "sale del resolve" << std::endl;
  ros::Time t1 = ros::Time::now();
  if (cost > 0.0){ 
    
    ROS_INFO("Path calculated. Expended time: %f. Cost: %f", (t1 - t).toSec(), cost);
    
    visualization_msgs::Marker m = a.getPathMarker(path);
//     visualization_msgs::Marker m = a.getPathMarker();
    m.header.frame_id = "/map";
    path_pub.publish(m);
  } 
  else{
//     visualization_msgs::Marker m = a.getModel().testIntegration(init, true);
//     m.header.frame_id = "/map";
//     ROS_INFO("Points: %d", (int) m.points.size());
//     path_pub.publish(m);
    ROS_INFO("Could not get a path in %f seconds", (t1 - t).toSec());
  }
  
  graph_pub.publish(a.getGraphMarker());
  
  
  
  ros::spin();
  
  return 0;
}