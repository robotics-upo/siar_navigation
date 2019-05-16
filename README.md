# SIAR_NAVIGATION

Planning navigation for SIAR robotics platform.

### Description

siar_navigation allows you to implement a navigation system based in a controller, costmap and a planner. The controller is base on `siar_controller` package and allows avoid positive and negative obstacles. The costmap is based on `siar_costmap` package and using a cost function compute the trajectory value considering positive and negative obstacles. The planner is based on `siar_planner` package and computes a path using the trajectories values from costmap . The planner can use the next algorithms to compute the path: RRT, bi-RRT, t-RRT and t-bi-RRT.

### Dependencies

To compile and execute successfully `siar_navegation` is necessary to compile the next packages:
  
* costmap-2d, navigation, gazebo-ros, gazebo-ros-pkgs, gazebo-ros-control and libann-dev.
```
sudo apt-get install ros-kinetic-costmap-2d
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-gazebo-ros
sudo apt-get install ros-kinetic-gazebo-ros-pkgs
sudo apt-get install ros-kinetic-gazebo-ros-control

sudo apt-get install libann-dev
```

From the repository `robotics-upo` are necessary the next packages:
 
* function [https://github.com/robotics-upo/functions.git]
* siar_package (banch kinetic) [https://github.com/robotics-upo/siar_packages.git]
* arduimu_v3 [https://github.com/robotics-upo/arduimu_v3.git]
* depth2cloud [https://github.com/robotics-upo/depth2cloud.git]


### ECMR 2019

For the publishing in ECMR (European Conference on Mobile Robots) 2019 the planner was testing in the siar_simulator, and previously in a synthetic map, where the planner with the best performance was t-RRT.

#### Synthetic map

To execute the synthetic map test you can follow two different ways: 

* First using: 
```
roslaunch siar_planner test_synthetic.launch
``` 
which allow you to execute just one planner in a problem , a number of time.

* Second using:
```
rosrun siar_planner test_all_algorithms.sh
```
which allow you to execute all the planners in all the problem, a number of time. E.g, for ten times `rosrun siar_planner test_all_algorithms.sh 10`.

#### Gazebo simulation

To execute Gazebo Simulation is necessary to follow the next steps:

1. Compile the siar_simulator package from [https://github.com/robotics-upo/siar_simulator]

2. Execute: 
```
roslaunch siar_gazebo siar_simulator_complete_T130_gut30.launch
``` 
to have a SIAR with seven cameras, or: 
```
roslaunch siar_gazebo siar_simulator_complete_T130_gut30_velodyne.launch
```
to have SIAR with six cameras and one velodyne. 
*IMPORTANT*: the launch in siar_simulator start in pause to avoid conflict in the spawn of the models Gazebo. This will generate a ROS_ERROR from `siar_costmap`, because is waiting to recieve the map. To finish with ROS_ERROR just push play in the simulation.

3. Exetuce:
```
roslaunch siar_planner planner_action_server_simulation.launch
``` 
To use differents planners change the parameter `planner_type`. Congratulations !!! now you are ready to navigate with SIAR in sewer environment.



