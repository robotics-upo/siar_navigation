# siar_navigation
Compilation of packages for navigation in sewers with the SIAR platform

This package allows you to implement a navigation system based in a controller, costmap and a planner. The controller is made to avoid positive and negative obstacles and, the costmap to evaluate in a cost function the positive and negative obstacles, values that are used in the planner to find the safe path. The planner that can be used are: RRT, bi-RRT, t-RRT and t-bi-RRT.

# ECMR 2019
For the ECMR (European Conference on Mobile Robots) 2019 the planner was testing in the siar_simulator, and previously in a synthetic map, where the planner with the best performance was t-RRT.

To execute the synthetic map test you can follow two different ways: 
- The first using: roslaunch siar_planner test_synthetic.launch, which allow you to execute just one planner in a problem , a number of time.
- The secod using: rosrun siar_planner test_all_algorithms.sh,  which allow you to execute all the planners in all the problem, a number of time. E.g, for ten times rosrun siar_planner test_all_algorithms.sh 10.

To execute Gazebo Simulation is necessary to follow the next steps:

1. Compile the siar_simulator package from> https://github.com/robotics-upo/siar_simulator

2. Execute: "roslaunch siar_gazebo siar_simulator_complete_T130_gut30.launch" to have a SIAR with seven cameras or "roslaunch siar_gazebo siar_simulator_complete_T130_gut30_velodyne.launch" to have SIAR with six cameras and one velodyne. 
IMPORTANT: the launch in siar_simulator start in pause to avoid conflict in the spawn of the models Gazebo. This will generate a ROS_ERROR from siar_costmap, because is waiting to recieve the map. To finish with ROS_ERROR just push play in the simulation.

3. Exetuce "roslaunch siar_planner planner_action_server_simulation.launch" (To use differents planners change the parameter "planner type"). Congratulations !!!, now you are ready to navigate with SIAR in sewer environment.


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
 
