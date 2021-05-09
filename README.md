# CS498 IR Final Project
Pick and place drone group project for CS498 IR

## Setup
### Docker
1. Install [Docker](https://docs.docker.com/get-docker/)
2. Run `docker build -t 498ros:drone .` in the repo folder
3. Run `docker run -it 498ros:drone`
4. Run `source ros_entrypoint.sh && source /opt/ros/noetic/setup.bash`
5. Run `cd ~/catkin_ws`
6. Run `catkin_make && source devel/setup.bash`
7. Run `nano src/ROS-TCP-Endpoint/config/params.yaml`
8. Edit the file so that it's
    ```yaml
    ROS_IP: 127.0.0.1
    ROS_TCP_PORT: 10000
    ```
9. (Skip this for now) Run `rosparam load src/ROS-TCP-Endpoint/config/params.yaml`
10. Run `roscore`
   
   ROS Core should now be started

   On future Docker runs you must run `docker container ls` and `docker exec -it NAME bash`

### Unity
1. Install [Unity](https://unity3d.com/get-unity/download)
