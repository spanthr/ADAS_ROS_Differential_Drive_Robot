Assignement 4: Obstacle Avoidance With Turtlebot3

Team: Bun Tikki

Team Members: Ankit Verma, Anshuman Sharma, Prateek Sharma, Rishabh Bhatia and Shaurya Panthri

Part 1: In this section we developed a code in python 2.7 around the problem of wall following. For this we implemented p-controller gain to achieve a constant distance difference of 0 units between the left wall and the right wall. We limited the field of view for the left side and right side of the lidar data, i.e. extracte 60 degree values both sides. We tried providing a constant value of velocity in linear direction. However constraints were applied during implementaiton of code in real world.

Part 2: In this section we developed the obbstacle avoidance algorithm. The approach presented in the question was adopted. 3 sectors were considered and calculated logic was implemented for the bot to provide steering ability to the bot in case of collision. The bot successfully follows a collision free path.

Part 3: The last part of the assignment was to implement codes in real world scenario. Many exceptions were observed. Since the walls made with cardboard boxes had gaps in between them, the lidar data was abrupt during such instances which led to improper velocities. Also, modifications to the orignal codes were made to adopt the real world parameters, such as wide optimization was done to get appropriate p-gain. Linear velocity was changed several times and the maximum distance that the turtlebot could have with the wall was incorporated and changed as well.
