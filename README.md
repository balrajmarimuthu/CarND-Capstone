# Project Overview
[image1]: ./sys_int.png "System Integration"
[image2]: ./sys_arch.png "System Architecture"
![System Integration][image1]
In this project, the goal is to write ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. 
# System Architecture
![System Architecture][image2]
## Waypoint Loader
This node loads the static waypoint data and publishes to `/base_waypoints`.
## Traffic Light Detector
This node publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.
## Waypoint Updater
The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.
## Waypoint Follower
This node contains code from [Autoware](https://github.com/CPFL/Autoware) which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic.
## DBW node
Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. The `dbw_node` subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

# Rubic points
## Running the Code
**The code is built successfully and connects to the simulator.**
No errors observed with `catkin_make`, `source devel/setup.sh` and `roslaunch launch/styx.launch` and connection to the simulator is established

## Control and Planning
**Waypoints are published to plan Carla’s route around the track.**
Waypoints are published to  `/final_waypoints`  in waypoint_updater.py (Line 82-115).
The limits for acceleration 10 m/s^2  and for jerk 10 m/s^3 are not exceeded.
The top speed of the vehicle  is limited to the km/h velocity set by the velocity rosparam in  `waypoint_loader`, which 40 kph.

**Controller commands are published to operate Carla’s throttle, brake, and steering.**
`dbw_node.py`  and `twist_controller.py` are implemented with PID controller for throttle, yaw controller for steering and low pass filters for both throttle and brake. The final commands are published to `/vehicle/throttle_cmd`, `/vehicle/brake_cmd` and `/vehicle/steering_cmd`

## Successful Navigation

**Successfully navigate the full track more than once.**
The vehicle is able to complete more than one full loop of the track without running off road or any other navigational issues.

## Reflection
Thanks to the Code walkthrough lessons, much of the effort happened in the `twist_controller.py` where the controllers parameters are tuned so that the motion commands do not cause any navigational issues. The optional task of traffic light classification is not tried. However, the ROS node implementation, pub-sub architecture and communication between nodes, use of rqt_console were some of the interesting things learned in this project.