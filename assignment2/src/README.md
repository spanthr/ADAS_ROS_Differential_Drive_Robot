The folder contains the three python circle.py, square_openloop.py, square_closed.py
The package assignment2 contains the src folder for the the codes.
All the codes are made executable using these nodes.

1. Circle_py: is the code that moves the turtle with constant twist velocity.
speed and distance are variables that together provide a constant velocity to the turtle using the Publisher.
These denote the linear and angular velocity of the turtle that are provided to the turtle sim node.

2. Sqaure_open loop: Roslaunch is used to run this code along with "assignment2" that is the node.
Two functions that is move() and rot() were created using the def command that provide the linear motion that is
equal to the length of the square and the other function rot() for angular motion of 90 degree at the edges. This code
generates a square oevement by the turtle in the turtlesim_node, a 2x2 sqaure with a linear velocity of 0.2 units 
and 0.2rad/s angular velocity.

3. Square_closedloop.py involves a code that moves the turtle to generate a sqaure 3x3 with velocity control. I used GotoGoal
as a refernce. I passed the cordinate points of the sqaure in form of an array to the code one by one using a for loop
in the main function(). An initializer/constructor creates the instance of the class and assign values to the variables.
To generate the required motion first of all, angular motion was generated using a while loop that checked if the difference 
of required angle and current angle (using Eucledian method, atan2 and absolute function) is greater than the tolerance 0.001.
Self.pose node was used to publish the position of the Turtle at every instance and update the statements.
Similar logic was used in the following while loop for providing velocity control in linear direction. Instead the loop here 
made the turtle move in linear direction and the check variable was again the position of the turtle until it reached the
desired cordinate. (Distance formula was used for calculation)

TUrtlesim was used to excecute the python script files.
