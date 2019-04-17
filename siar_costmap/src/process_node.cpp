#include <ros/ros.h>
#include <siar_costmap/process_costmap.hpp>

int main( int argc, char **argv)
{
	// Setup ROS
	ros::init(argc, argv, "process_costmap_node");
	
	// Create service
	ProcessCostmap process_costmap;
	
	// Spin forever
	ros::spin();
    
	return 0;
}
