# SIAR_NAVIGATION

Planning navigation for SIAR robotics platform.

### Description

siar_navigation allows you to implement a navigation system based in a controller, costmap and a planner. The controller is implemented in the `siar_controller` package and allows avoid positive and negative obstacles. The costmap is implemented in `siar_costmap` package and detects positive and negative obstacles from sensor data and provides the user with a cost map taking into account the position of the obstacles. The planner is implemented in the `siar_planner` package and computes a path taking into account a costmap. The planner can use the next algorithms to compute the path: RRT, bi-RRT, t-RRT and t-bi-RRT.

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

Here you can find the instruction for executing the simulations presented in the paper "Sampling-based planning for a sewer inspection ground robot" in the European Conference on Mobile Robots (EMCR 2019). The four different planners were tested in the in a testbench of simulations in synthetic maps, where the planner with the best performance was t-RRT. Finally, this t-RRT planner in realistic simulations using the Gazebo high fidelity robot simulator. The model and world used in the simulations can be found at [https://github.com/robotics-upo/siar_simulator].

All the code was developed and tested in Ubuntu 16.04 and ROS Kinetic. 

#### Synthetic map

Run the following command to execute all the synthetic map simulations presented in the paper: 

```
rosrun siar_planner test_all_algorithms_synthetic.sh <number_of_tests>
```
It allows you to execute all the proposed tests in the scenarios a given number of times. E.g, for ten times:

```
rosrun siar_planner test_all_algorithms.sh 10
```

#### Gazebo simulation

To execute the Gazebo simulations, it is necessary to follow the next steps:

1. Download the siar_simulator package into a ROS workspace from [https://github.com/robotics-upo/siar_simulator]

2. Compile the ROS workspace

```
catkin_make
```

3. Execute: 
```
roslaunch siar_gazebo siar_simulator_complete_T130_gut30.launch
``` 
to have a SIAR with seven cameras, or: 
```
roslaunch siar_gazebo siar_simulator_complete_T130_gut30_velodyne.launch
```
to have SIAR with six cameras and one velodyne. 

*IMPORTANT*: this launchers will start the simulator in pause mode to avoid conflict when spawing the different models. The `siar_costmap` module will generate some errors that will no longer thrown just by pushing play in the Gazebo simulator.

4. Execute:
```
roslaunch siar_planner planner_action_server_simulation.launch
``` 
To use differents planners just change the parameter `planner_type` from to one of the following options: 'rrt', 'trrt', 'birrt', 'tbirrt'. 

Congratulations !!! now you are ready to navigate with SIAR in sewer environment.



