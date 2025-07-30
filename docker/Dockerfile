FROM osrf/ros:melodic-desktop-full

# Create workspace and download files
RUN apt-get update && apt-get install git -y

RUN cd ~/ && git clone https://github.com/david-alejo/linux_install && apt-get install cmake

RUN git config --global credential.helper store
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/melodic/setup.sh
RUN apt install ros-melodic-costmap-2d ros-melodic-amcl ros-melodic-map-server ros-melodic-urg-node -y
RUN apt update && apt upgrade -y && apt install ros-melodic-aruco xterm gdb -y


# Required ROS packages  
RUN sudo apt update && apt install -y ros-melodic-joy ros-melodic-teleop-twist-joy \
    ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
    ros-melodic-rgbd-launch ros-melodic-rosserial-arduino \
    ros-melodic-rosserial-python ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
    ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
    ros-melodic-compressed-image-transport ros-melodic-rqt* ros-melodic-rviz \
    ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers \
    ros-melodic-vision-msgs ros-melodic-urg-node ros-melodic-rplidar-ros

# ROS packages for Turtlebot3 robot     
RUN sudo apt install -y ros-melodic-dynamixel-sdk \
    ros-melodic-turtlebot3-*

# We also install Git and other tools
RUN sudo apt install -y git vim nano iputils-ping net-tools

# MESA drivers for hardware acceleration graphics (Gazebo and RViz)
RUN sudo apt -y install libgl1-mesa-glx libgl1-mesa-dri libx11-6 

# Multiple terminals support (tmux)
RUN apt install -y tmuxinator

# By default the DockerFile uses the root as user.
# We will add a custom user so that we work in user space by default
# And then switch to that user
# Source: https://www.docker.com/blog/understanding-the-docker-user-instruction/
RUN useradd -ms /bin/bash siar
# Sometimes it is good to add a password so that we can use sudo to install things
# To this end, uncomment the following line, remove extra ### and change the password
RUN echo "siar:siar" | chpasswd
### Remove #### to change the password
RUN adduser siar sudo & adduser siar dialout & adduser siar plugdev
RUN usermod -aG dialout siar  && usermod -aG plugdev siar  && usermod -aG sudo siar

# For the real robots. Please see how to share USB from windows via usbipd
RUN apt -y install usbutils udev ros-melodic-joy libann-dev


USER siar

# We will use the home directory as work directory
# So that the docker starts at HOME!
WORKDIR "/home/siar"


RUN mkdir -p ~/catkin_ws/src/ && cd ~/catkin_ws/src && git clone https://github.com/robotics-upo/arduimu_v3.git && git clone https://github.com/robotics-upo/libelium_waspmote_gas_node.git && ls ~/catkin_ws/src
RUN cd ~/catkin_ws/src && git clone https://github.com/robotics-upo/siar_navigation.git && git clone https://github.com/robotics-upo/functions && git clone https://github.com/robotics-upo/siar_packages.git
RUN cd ~/catkin_ws/ && source /opt/ros/melodic/setup.sh && catkin_make && catkin_make install
RUN cd ~/catkin_ws/src && git clone -b melodic https://github.com/robotics-upo/siar_simulator.git

RUN cd ~/catkin_ws  && source ~/catkin_ws/devel/setup.bash && catkin_make



