Package name: Assignment3_turtlebot3

This package contains two launch files.

(I) move.launch: To run this file

	$ roslaunch assignment3_turtlebot3 move.launch code:=circle

The above command will launch gazebo empty world with turtlebot3 burger and the bot will follow a circular trajectory.

	$ roslaunch assignment3_turtlebot3 move.launch code:=square

The above command will launch gazebo empty world with turtlebot3 burger and the bot will follow a square trajectory of side 2. Since the algorithm is openloop, therefore the square would not follow the exact trajectory.


(II) turtlebot3_wall_launch.launch: To run this file
 
	$ roslaunch assignment3_turtlebot3 turtlebot3_wall_launch.launch

The above command will launch a new world called turtlebot3_wall.world which contains a wall. The bot will start moving towards the wall and after a certain distance from the wall the turtlebot would stop to avoid collision.



