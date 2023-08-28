# Husky robot navigation

## Authors: Jacopo Raffi - Simone Marzeddu

A Husky robot has to navigate a maze using its on-board camera. The robot moves towards visual cues to guide it towards the exit.

## How to run

Executing the following commands in order will start the `Gazebo` simulation:

* `roscore & sleep 1 ; rosparam set use_sim_time true` -> start ROS master node,

* `rosrun gazebo_ros gzserver braitenberg_nav_project/maze.sdf` -> set up the simulation environment in Gazebo,

* `gzclient` -> activate a visual representation of the simulation,

* `cd braitenberg_nav_project/` -> set the Linux shell path inside the main folder,

* `python3 -m sm.lights_switcher` -> generate human targets in the simulation environment,

* `cd ROS_Plugins/catkin_ws/src/` -> set the Linux shell path inside the ROS_Plugins/catkin_ws/src/ folder,

* `rosrun robot_vision yolo_node.py` -> start the robot vision module,
    
* `rosrun robot_control pid_controller.py` -> start the robot control module.
