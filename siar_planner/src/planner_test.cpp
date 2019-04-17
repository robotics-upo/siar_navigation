#include "siar_planner/Planner.hpp"
#include "siar_planner/rrt.hpp"
#include "siar_planner/trrt.hpp"
#include "siar_planner/biRRT.hpp"
#include "siar_planner/tbiRRT.hpp"
#include "ros/ros.h"
// #include <functions/functions.h>
#include <iostream>
#include <sstream>
// #include <iomanip>
#include <string.h>
#include <visualization_msgs/MarkerArray.h>

using functions::RealVector;
// using namespace std;

int main(int argc, char** argv){
  
  ros::init(argc, argv, "test_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::ofstream ofs;
  std::string output_file;
  
  Planner *planner = NULL;
  std::string planner_type;
  pnh.param("planner_type", planner_type, std::string("rrt"));
  
  if (planner_type == "trrt") {
      ROS_INFO("Planner type: tRRT");
      planner = new tRRT(nh, pnh);
  } else if (planner_type == "tbirrt") {
      ROS_INFO("Planner type: t-bi-RRT");
      planner = new tbiRRT(nh, pnh);
  } else if (planner_type == "birrt") {
      ROS_INFO("Planner type: bi-RRT");
      planner = new biRRT(nh, pnh);
  } else {
      ROS_INFO("Planner type: RRT");
      planner = new RRT(nh, pnh);
  }

  ROS_INFO("Waiting for map initialization");
  while (!planner->getModel().isInit() && ros::ok()){ 
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
  pnh.param("output_file", output_file, std::string ("~/test.txt"));
  int n_tests;
  pnh.param("n_tests", n_tests, 1);
  
//   pnh.param("output_file", output_file);
  
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
  ros::Publisher path_pub = nh.advertise<visualization_msgs::MarkerArray>("path_marker", 2, true);
  
  ROS_INFO("Publishing visualization markers (init and goal)");
  vis_pub.publish(planner->getModel().getMarker(init));
  goal_pub.publish(planner->getModel().getMarker(goal, 1));
  
  ofs.open(output_file.c_str(), std::ofstream::app);
  for (int cont = 0; cont < n_tests && ros::ok(); cont++) {

    ROS_INFO("Test number: %d", cont);
    ros::Time t = ros::Time::now();
    double cost = planner->resolve(init, goal, path);
    ros::Time t1 = ros::Time::now();
    if (cost > 0.0){ 
        
        ROS_INFO("Path calculated. Expended time: %f. Cost: %f", (t1 - t).toSec(), cost);
        
        visualization_msgs::MarkerArray m = planner->getPathMarker(path);
        path_pub.publish(m);
    } 
    else{
        ROS_INFO("Could not get a path in %f seconds", (t1 - t).toSec());
    }
    
    graph_pub.publish(planner->getGraphMarker());
    
    

    if (ofs.is_open()) {
        std::cout << "Guardado en archivo de salida: " << output_file << std::endl;
        ofs << (t1 - t).toSec() << "," << path.size()* planner->getDeltaT() << "," << path.size() 
        << "," << planner->getGraphSize()
        << "," << planner->retCostPath(path) 
        <<std::endl;
    } 
    else {
        std::cout << "No se puede abrir el archivo de salida" << std::endl;
    }
  }
  ofs.close();
    
  return 0;
}

