Course: Control of Mobile Robots, professor Bascetta Luca, A.Y. 2023-2024 
Scope: Homework 2, unicycle kinematic simulator
Author: William Stucchi


Description:
This is the second homework of the course Control of Mobile Robots, held by professor Bascetta Luca.
The scope of this project was to introduce the students in the use of ROS, in order to acquire
a decent understanding of basic concepts such as nodes, topics and parameter server.
The student were asked to provide a ROS package containing the description of 3 nodes:

	- system_clock node: this node has the role of producing a clock for the counter nodes, that 
				thus synchronize their operations based on the frequency of the clock
				provided by this node. 
				The computation of this node is published to topic /clock.
 
	- counter node: this node increments a counter, starting from a given value (0 in this specific case)
			until it reaches a threshold (10 in this specific case). After that, the value of the
			counter is published in the topic /value.
			A number N = 2 of this nodes must be active in the system when launched.
			This node sychronizes its operations by subscribing to topic /clock and reading
			the clock published by the system_clock node.

	- sumup node: this node listend on topic /value and when it receives 2 values of the counters 
			(provided by the two active counter nodes) performs its operations, that include
			the summation of the two counter values received and the multiplication of the 
			result by a value alfa (1 in this case).
			The result is the published in the topic /sum and as ROS_INFO.

	- parameter file: in the src/config folder it is provided a configuration file containing the parameter
			that the system_clock node needs to read from the parameter server. The value is used
			by the node in order to increment the value of the clock.

	- project.launch: this file is the launch file of the project. It set the parameter server and the it 
			runs the system_clock node, the two counter nodes and the sumup node.


How to run:
Provided that ros melodic is installed in the system and that the ROS workspace is correctly sourced.
Copy the entire ros_basics package in the your_workspace/src folder, the run catkin_make in the environment.

Open a terminal and run the command: 
	- roslaunch ros_basics project.launch

Optionally, open 3 more terminals and run the commands:
	- in terminal 1: rostopic echo /clock
		this command shows the content of the topic /clock, where system_clock node is publishing.

	- in terminal 2: rostopic echo /value
		this command shows the content of the topic /value, where the two counter nodes will publish.

	- in terminal 3: rostopic echo /sum
		this command shows the content of the topic /sum, where sumup node will publish.

