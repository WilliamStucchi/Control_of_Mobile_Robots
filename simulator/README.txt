Course: Control of Mobile Robots, professor Bascetta Luca, A.Y. 2023-2024 
Scope: Homework 2, unicycle kinematic simulator
Author: William Stucchi


DESCRIPTION:
This is the second homework of the course Control of Mobile Robots, held by professor Bascetta Luca.
The scope of this project was to simulate a CIRCULAR trajectory performed by the robot, taking into 
account the actuations of the robot.
The student were asked to provide a ROS package containing the description of 2 nodes:

	- robot simulator: this node takes inputs from the topic \cmd, of type Float64MultiArray, representing
                        the actual time and robot commands, and publishes on topic \state a Float64MultiArray
                        representing the actual time and robot state.
 
	- test node: this node publishes robot inputs on topic \cmd to drive the robot along the desired trajecotry.

In addition, it was requested to include a sim_robot.launch file, a parameter server with robot and trajectory parameters
and a python script that loads a bag and shows the following figures:

    - the desired trajectory, as a dashed red line, and the actual trajectory, as a solid blue line.
    - the robot commands wrt time (linear velocity and angluar velocity).
    - the robot state wrt time (x, y, and theta).

The actuation of the linear velocity was a unitary transfer function, while the angluar velocity is actuated 
by the following transfer functions: TF(s) = 8100 / (s^2 + 135*s + 8100).

HOW TO RUN:
Provided that ros melodic is installed in the system and that the ROS workspace is correctly sourced.
Copy the entire simulator package in the your_workspace/src folder, then run catkin_make in the environment.

Open a terminal and run the command: 
	- roslaunch simulator sim_robot.launch

In the project folder there is already provided a bag that was recorded from previous runs, while launching the 
project from ros will record another bag. 

Optionally, open 2 more terminals and run the commands:
	- in terminal 1: rostopic echo /cmd
		this command shows the content of the topic /cmd, where actuation commands are publihed by the test node.

	- in terminal 2: rostopic echo /state
		this command shows the content of the topic /state, where the state of the robot is published.

SCRIPT:
The python script that read the bag and show figures is located in the simulator/script folder.
To run it open a new terminal, locate the script and the bag, then write:
    - python <script_path> <bag_path>

3 figures will be shown.

