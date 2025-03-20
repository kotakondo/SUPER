FROM ros:noetic

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 20.04, ROS Noetic for SUPER"

ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

RUN sudo rm -rf /var/lib/apt/lists/*
RUN apt-get update && \
      apt-get upgrade -y && \
      apt-get install -q -y --no-install-recommends git apt-utils wget tmux vim tmuxp make && \
      apt-get autoremove -y && \
      apt-get clean

RUN sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
RUN sudo apt-get install -y ros-${ROS_DISTRO}-mavros-msgs
RUN sudo apt-get install -y ros-${ROS_DISTRO}-tf2-sensor-msgs
RUN sudo apt-get install -y ros-${ROS_DISTRO}-control-toolbox 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-ros-control 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-robot-localization 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-lms1xx 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-interactive-marker-twist-server 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-hector-gazebo-plugins 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-move-base 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-ros-controllers 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-pointgrey-camera-description 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-hardware-interface 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-message-to-tf 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-control
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-pcl-ros
RUN sudo apt-get install -y ros-${ROS_DISTRO}-rqt-gui
RUN sudo apt-get install -y ros-${ROS_DISTRO}-rqt-gui-py
RUN sudo apt-get install -y ros-${ROS_DISTRO}-rviz
RUN sudo apt-get install -y python3-pip
RUN sudo apt-get install -y python3-catkin-tools

RUN sudo apt-get install -y libglfw3-dev 
RUN sudo apt-get install -y libglew-dev
RUN sudo apt-get install -y libeigen3-dev       
RUN sudo apt-get install -y libdw-dev
RUN sudo apt-get install -y ros-${ROS_DISTRO}-mavros*
RUN sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-pcl*
RUN sudo apt install -y ros-${ROS_DISTRO}-rosfmt
RUN sudo apt-get install -y libncurses-dev
RUN sudo apt-get install -y ros-${ROS_DISTRO}-rqt-graph
RUN sudo apt-get install -y libignition-math4-dev
RUN sudo apt install -y cmake
                            
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
RUN python3 -m pip install pyquaternion

# Set up SUPER workspace
RUN mkdir -p /home/kota/super_ws/src
WORKDIR /home/kota/super_ws/src


# Install dependencies
# Livox-SDK2
WORKDIR /home/kota/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /home/kota/Livox-SDK2
RUN mkdir build
WORKDIR /home/kota/Livox-SDK2/build
RUN cmake .. && make -j
RUN sudo make install

# acl-gazebo
WORKDIR /home/kota/super_ws/src
ARG DUMMY=unknown
RUN DUMMY=${DUMMY} git clone https://gitlab.com/mit-acl/lab/acl-gazebo.git
WORKDIR /home/kota/super_ws/src/acl-gazebo
RUN git checkout noetic
WORKDIR /home/kota/super_ws/src
# snapstack_msgs
RUN git clone https://gitlab.com/mit-acl/fsw/snap-stack/snapstack_msgs.git

# Install SUPER
WORKDIR /home/kota/super_ws/src
RUN DUMMY=${DUMMY} git clone https://github.com/kotakondo/SUPER.git
# Use ROS1 Noetic
RUN bash /home/kota/super_ws/src/SUPER/scripts/select_ros_version.sh ROS1
# Build SUPER
WORKDIR /home/kota/super_ws

# RUN sudo apt install -y ros-${ROS_DISTRO}-ament-cmake
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DBUILD_TYPE=Release"

# livox_ros_driver2
RUN mkdir -p /home/kota/livox_ros_ws/src
WORKDIR /home/kota/livox_ros_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git
WORKDIR /home/kota/livox_ros_ws/src/livox_ros_driver2
RUN ./build.sh ROS1

# Mid360_simulation_plugin
RUN mkdir -p /home/kota/mid360_ws/src
WORKDIR /home/kota/mid360_ws/src
RUN git clone https://github.com/fratopa/Mid360_simulation_plugin.git
WORKDIR /home/kota/mid360_ws
RUN bash -c "source /opt/ros/noetic/setup.bash && source /home/kota/super_ws/devel/setup.bash && catkin_make -DBUILD_TYPE=Release"

# Set up bashrc
RUN echo "source /home/kota/super_ws/devel/setup.bash" >> ~/.bashrc

# Set up ROS
# RUN echo "export ROS_MASTER_URI=http://192.168.0.1:11311" >> ~/.bashrc
# RUN echo "export ROS_MASTER_URI=http://172.17.0.2:11311" >> ~/.bashrc
# RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
RUN echo "export ROS_IP=localhost" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc

# shortcut 
RUN echo "alias tks='tmux kill-server'" >> ~/.bashrc
RUN echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
RUN echo "alias eb='code ~/.bashrc'" >> ~/.bashrc
RUN echo "alias gs='git status'" >> ~/.bashrc
RUN echo "alias gp='git push'" >> ~/.bashrc
RUN echo "alias roscd='cd /home/kota/super_ws'" >> ~/.bashrc
RUN echo "alias cb='roscd && catkin build && sb'" >> ~/.bashrc
RUN echo "alias ss='roscd && source devel/setup.bash'" >> ~/.bashrc
RUN echo "alias cbf='clear && roscd && catkin build && ss'" >> ~/.bashrc

# Set up Entry Point
# ##########################################
COPY ./super_sim.yml /
RUN chmod +x /super_sim.yml
ENTRYPOINT ["tmuxp", "load", "/super_sim.yml"]

