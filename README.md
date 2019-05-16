# siar_navigation
Compilation of packages for navigation in sewers with the SIAR platform

This package allows you to implement a navigation system based in a controller, costmap and a planner. The controller is made to avoid positive and negative obstacles and, the costmap to evaluate in a cost function the positive and negative obstacles, values that are used in the planner to find the safe path. The planner that can be used are: RRT, bi-RRT, t-RRT and t-bi-RRT.

For the EMCR (European Conference on Mobile Robots) 2019 the planner was testing in the siar_simulator, and previously in a synthetic map, where the planner with the best performance was t-RRT.

# Compilation and execution package in simulation

To compile and execute successfully siar_navegation is also necessary to download the next packages from GitHub robotics-upo:
 
 - function (git clone https://github.com/robotics-upo/functions.git)
 - siar_package (git clone -b kinetic https://github.com/robotics-upo/siar_packages.git)
 - arduimu_v3 (git clone https://github.com/robotics-upo/arduimu_v3.git)
 - depth2cloud (git clone https://github.com/robotics-upo/depth2cloud.git)

The next ROS packages:
  
 - costmap-2d, navigation, gazebo-ros, gazebo-ros-pkgs , gazebo-ros-control.
 

And the next package:

 - libann-dev.
