# Robot control in Gazebo/Rviz simulation

This repository contains code to run a gazebo simulation of a moving, physical robot, along with various tools to move it and interact with the simulated environment. Once the simulation is running, it is possible to interact with it with an external terminal that allows the user to input the robot's linear and angular velocities to move it in the environment for 4 seconds at a time. 
While moving in the simulation, the robot can encounter obstacles of various shapes, and can obtain its distances through the /scan package. A controller is implemented in such a way that when the robot encounters an obstacle from a certain threshold distance it will go back to the previous position it was in. A service is also integrated to change this threshold distance at user's please. 
Other features include a custom message published every five seconds that tells the user the position and direction of the closest obstacle and a service that keeps track of the user inputs and publishes the average velocities from the last five inputs.

## Code explanation and structure 
The project's src folder is composed of three packages, each containing specific features:

- a2_rt_controller, a python package that contains the distance controller (*robot_controller.py*), the terminal interface for user inputs (*terminal_interface.py*) and a minimal subscriber to the custom message that publishes the closest obstacle (*custom_msg_subscriber.py*);

- a2_rt_interfaces, a cmake package the provides the custom message and service files, stored in the msg and srv folders respectively, and two service nodes that allow to change the threshold and show the last five inputs' average values (*avg_vel_service.cpp, threshold_service.cpp*);

- a2_rt_simulation, which provides the Gazebo/Rviz simulation environment.

Each one of these packages is provided with a launch file stored in a specific launch folder to reduce the amount of running operations needed.

## How to run
Requirements: ROS2 setup, Gazebo, Rviz.
PRoject tested on Docker image tiryoh/ros2-desktop-vnc:jazzy-20251019T1559

Procedure to run the system:
- Three terminals needed, one for every package. It is essential for the project's functionality that all packages are run together. 
- On each terminal, use this two-command sequence:
  ```
  colcon build
  source install/setup.bash
  ```
- Run these three commands to launch all the project features (one on each terminal):
  ```
  ros2 launch a2_rt_controller robot_control.launch.py
  ros2 launch a2_rt_interfaces service_nodes.launch.py
  ros2 launch a2_rt_simulation spawn_robot.launch.py
  ```
- While running, you can experiment with the various terminals opened by the launch files.
- Enjoy!

## Author
Sisani Francesco