#include "siar_controller/siar_footprint.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace siar_controller;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_footprint");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("footprint_marker", 10, true);
  
  double cellsize = 0.02;
  // POINTS markers use x and y scale for width/height respectively
  
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double width = 0.8;
  double x_elec = 0.0;

  if (argc > 1) {
    if (strlen(argv[1]) > 1 && strcmp(argv[1], "-h") == 0) {
      std::cout << "Usage: " << argv[0] << " <width> <x_elec> <x> <y> <theta>\n";
      return 0;
    }
    width = atof(argv[1]);
  }
  if (argc > 2) {
    x_elec = atof(argv[2]);
  }
  if (argc > 3) {
    x = atof(argv[3]);
  }
  if (argc > 4) {
    y = atof(argv[4]);
  }
  if (argc > 5) {
    theta = atof(argv[5]);
  }
  SiarFootprint fp(cellsize, 0.8, width, 0.08, true, x_elec);
  
  visualization_msgs::Marker m;
  fp.addPoints(x, y, theta, m, 0, true);
  fp.addPoints(x+2, y, theta,m, 0, false);
  
  marker_pub.publish(m);
  sleep(1);
  ros::spinOnce();
  sleep(2);
//   fp.printFootprintCollision(x, y, theta, marker_pub, 1);
  
  ros::spin();
 
  return 0;
}
