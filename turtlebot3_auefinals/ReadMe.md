AuE 8930: Autonomy Sci. and Systems final project readme
Team2: Buntikki

Team Members
	Prateek Sharma
	Anshuman Sharma
	Rishabh Bhatia
	Ankit Verma
	Shaurya Panthri

Member Contributions: 
	Prateek Sharma: Decoding msg structure for tiny yolo to extract to stop sign ID. Maximizing bot’s speed for minimizing time. Installing darknet_ros (tiny YOLO) and
 	people_detection packages and respective dependencies. 
	Anshuman Sharma:  Line following and stop sign detection. Proportional controller tuning and Tiny YOLO camera feed resolution optimization for better frame rates.
 	Code integration from line following to Human tracking.
	Rishabh Bhatia: Developing trigonometric equations for Human tracking. Understanding how to implement trained models. Identifying potential fallbacks in the code and
 	tuning the PID.
	Ankit Verma: April tag detection and April tag implementation and tuning, April tag modelling in Gazebo world (placement and orientation), code integration. Installing
 	April tag library and apriltag_ros package.
	Shaurya Panthri: Wall following and obstacle avoidance tuning the Proportional controller and optimizing LIDAR sensing sectors and code integration. Combining the code for
 	wall following and obstacle avoidance to avoid switching.

Files included
	Scripts: 
		Move_combined_new.py
		Move_robot.py

	Launch files:
		Turtlebot3_autonomy_final.launch
		Leg_detection.launch
		April_tag_detection.launch

Running the simulation
	~$ roslaunch turtlebot3_auefinals turtlebot3_autonomy_final.launch

Tasks
Task 1 and Task 2: Wall Following and obstacle avoidance
	In the wall following task the turtlebot was expected to follow the wall and avoid collisions with it. Steering and forward velocities were calculated based on the sensed
	distance to the walls. 
	To avoid the need for April tag detection and code switching, the wall following, and obstacle avoidance code was combined into one. In the obstacle avoidance task, the bot
	was expected to navigate around obstacles smoothly without hitting any obstacles and go towards line following. As soon as bot detected the April tag within a distance of 0.2m
	the code switched to line following via use of flags. The key parameters optimized for this task are listed below.
 
	Front sensing:
		Front sensing sector: 340 deg to 20 deg 
		Linear velocity (without front obstacle): 0.2 m/sec
		Linear velocity (with front obstacle): 0.01 m/sec
	
	Side Sensing:
		Left sensing sector: 20 deg to 60 deg
		Right sensing sector: 300 deg to 340 deg
		Angular velocity proportional gain (without front obstacle): 2.5
		Angular velocity proportional gain (with front obstacle): 2.0



Task 3: Line following and stop sign detection
	In this task, the bot was expected to follow a curvy yellow path marked in the Gazebo environment. The camera video feed can be obtained as a series of images through the
	camera call back function. Thus, each image was then obtained nad processed further. A narrow band (was created in the hsv format) in which the color of the path was expected
 	to lie. Only color in this range was kept and the rest was masked. The center of this unmasked image was extracted, and a circle drawn with it as the center was overlayed 
	on this image. As the turtlebot moved forward one could see this blob move from side to side.
	
	For stop sign detection the bot was expeted to stop for 3 seconds when the stop sign was dectected. Trained models were picked up from real time Object Detection system
	(Tiny YOLO). Darknet_ros was used to identify objetcs using the trained models. The stop sign id was foind to be 11 Thus while running the line follower code, the program
 	continuously monitored the vide feed with the trained model to identify the stop sign. As soon as the bot detected a stop sign, it stopped for 3 seconds before it started 
	moving again. Flag variables were used to prevent the bot from stopping again should it detect the stop sign again.
 
	To avoid the need for April tag detection and a smoother code switching from line following to people tracking, a different approach was used. As soon as the yellow 
	line vanishes from the turtlbeot’s camera feed the code switches to human tracking directly. When there is a line to follow in the turtlbeots camera feed, the x and y 
	coordinates of the blob are returned. In the absence of the line, there is no blob so the center of the image is give as x and y coordinates (thus the bot will head 
	straight) and a flag value is changed. This flag value is later used to select between line following and people tracking code.

	The key parameters optimized for this task are listed below.
		Linear velocity: 0.08 m/sec
		Angular velocity proportional gain: 1/1400

Task 4: Human detection
	In this task the humanoid object is placed at a certain position within the environment. This humanoid can be moved and rotated about any direction and it is expected
	that the turtlebot follows this humanoid and stops at a certain distance behind it.
	The humanoid object can be tracked either via lidar or via camera or both subjected to results. In case of detection by Lidar, it becomes obvious that the two legs will
	be seen as a cluster of dots in a 2 U shapes. Its is obvious that to take into account sensor inaccuracies and different orientations a large range of points could classify
 	as legs. This calls for reference to trained data models which could identify a cluster of points detected by Lidar as a pair of legs and give out corresponding coordinates
 	of its center.

	Once the X and Y coordinates of the legs are obtained the next task is to make the bot follow it. Following can be seen as two separate processes.

		1. Orientating the bot in line with the person
		The Lidar sensor detected the legs with respect to the world frame. Thus, from the Lidar we got X and Y coordinates of the legs. The current X and Y cordinates
 		of the bot can be read from the /odom topic. Thus, as the angle f can be calculated by taking the Tan inverse of the coordinates of person and bot.

		Now that f is obtained, Ø needs to be calculated in order to calculate the angle ?. 
		It is obvious here that ? needs to be minimized. Thus, a P controller was used that set angular velocity in the Z component proportional to the angle ?.
		
		2. Keeping the person within specified distance range
		Quite simply the distance was calculated between the person and the bot.
		Again, a P controller was used to control the velocity and the bot was stopped as soon as the distance came below a threshold distance.
		Both these approaches when used together and upon tuning yielded the desired result and the bot followed the person without any problems.




