FROM ros:noetic-ros-base-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-robot=1.5.0-1* \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git /root/catkin_ws/src/ROS-TCP-Endpoint
