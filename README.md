# rainbow

# Introduction
Our project aims at the realization of a robot that has as objective the collection of some balls
scattered around the simulated environment following a specific order. In particular, we have chosen to base
our priority on the colors of the rainbow using the five colors of Newton instead of the seven of the complete rainbow
to simplify the world, which is already quite complex to render and run in the simulation phase.
simulation.
It has been chosen to use the Webots platform because open source and that one used during the exercises
carried out by engineer Lanza.
The task of the robot is that one to go to recover the balls around the map and to avoid obstacles if
arise. Once each ball has been retrieved, it will be the robot's goal to bring it to a specific point on the map where there will be a slide.
of the map where there will be a funnel-shaped chute. Inside this chute will be
the balls will be grouped in order, generating a rainbow as the final effect.

# Robot anatomy
We have chosen not to use the default Webots robots, but to build one manually using the
assets that the platform provides.
Regarding the sensors used, we chose to take advantage of:
- Inertial Unit. It simulates a sensor that measures the relative angles of roll, pitch and yaw. The
We used it mainly to calculate the yaw angle of the robot with respect to the Webots reference axes.
reference axes of Webots.
- GPS. Simulates a positioning sensor that measures absolute position in the Webots coordinate system.
Webots.
- Camera. Simulates a typical RGB camera in which the color recognition feature has been enabled.
colors.
- Distance sensors, two of which are useful for detecting obstacles and one for giving input to the arm to
close at the moment in which it is in possession of the ball.
4
As for the structure, we have chosen to create:
- Three arms, two of which are fixed to prevent the ball from escaping and one that rotates thanks to the use of a
hinge that allows only a rotational movement around a given axis. Specifically we have
chosen the Z axis as the axis on which to rotate the rotational motor.
- A body, to give the robot a visual appearance.
- Four drive wheels with a rotational motor in each of them to allow the robot to
move.

# Structure of the environment
We chose to use the default settings provided by Webots and to change only the basicTimeStep parameter to a value of 16.
basicTimeStep parameter, setting it to a value of 16. In this way it will come increased the accuracy and
stability of the simulation, especially for the physical calculations and the detection of the collisions.
The environment is composed as follows:
- Five balls of different colors in order to obtain a rainbow effect at the end of the collection
of them. We have also decided to set the recognitionColors parameter to the corresponding color
to allow the camera to recognize them.
- Four walls, to delimit the environment of action of the robot.
- A platform, useful for bringing the robot in front of the ramp from any direction it comes from.
- A ramp, where a funnel-shaped chute has been placed for collecting balls.
- An obstacle for the robot to avoid during its movements.

# Controller
It has been chosen to set up a TimeStep equal to the basicTimeStep of the atmosphere in order to synchronize the actions of the
robot to the execution of the simulation. In particular, we have recalled the function
wb_robot_step(TimeStep) every time a movement of the robot was executed.
In order for the robot to perform various tasks, we created five different states:
1. Detect_ball. Initial state of the robot in which it starts rotating on itself, counterclockwise, until it detects the ball of the same size.
until it detects the ball of the same color passed to the function. Once detected, it is
centered with respect to the camera view with a degree of precision of ±4px.
5
2. Grabs_ball. The robot picks up the ball in front of it avoiding obstacles if they are
obstacles along the way. Once the robot senses that it has the ball, it stops and closes its arm to catch it.
closes its arm capturing it.
3. Move_to_platform. It acquires the coordinates of the robot's starting point via GPS and calculates the
3. It acquires the coordinates of the robot's starting point via GPS and calculates the distance and direction in which the robot must rotate to reach the platform using odometry,
avoiding obstacles if any are present.
4. Move_to_ramp. Similar to the previous state, but the final goal is to reach the ramp.
the ramp.
5. Release_ball. Once the ramp is reached, the ball is released and slid onto the funnel-shaped
funnel shape. Finally, the robot moves back until it reaches the platform again.
To increase the readability and reusability of the code, functions have been implemented that can
perform elementary tasks such as
- move_robot;
- move_arm;
- enable_sensors;
- recognition_color;
- is_that_color;
- odometry;
- goal_oriented;
- distance_to_point;
- avoid_obstacle.

## void move_robot (const char *command)
This function allows to set the speed of every single wheel of the robot initializing it at initial speed
initial speed of 0.5 rad/s.
Depending on the string that is passed to the function, the robot can perform the following movements:
- LEFT: to rotate the robot to the left, the speed of the left wheels is set to -0.5 rad/s,
leaving the speed of the right wheels unchanged.
- RIGHT: to make the robot rotate to the right, the speed of the right wheels is set to -0.5 rad/s,
leaving the speed of the left wheels unchanged.
6
- GO STRAIGHT: to move the robot forward, the speed of all four wheels is set to 3.0
rad/s.
- GO BACK: to move the robot backwards the speed of all four wheels is set to -3.0
rad/s.
- STOP: to stop the robot, the speed of all four wheels is set to 0 rad/s.

## void move_arm (const char *command)
This function allows you to set the movement of the mechanical arm.
Depending on the string that is passed to the function, the robot can perform the following movements:
- CLOSE: the arm is rotated 90 degrees by the rotational motor, setting the variable
variable to -1.57 radians, therefore equivalent to -π/2.
- OPEN: the arm is returned to its initial position by the rotational motor, setting the rotation variable to 0 radians.
rotation variable to 0 radians.
void enable_sensors (void)
This function allows to activate all the sensors in the robot such as:
- GPS;
- inertial unit;
- distance sensors;
- rotational motor of arm and wheels;
- camera.
It is used inside the main function allowing the use of all sensors from the beginning.

## char* recognition_color(const double *color)
This function allows to recognize the color passed in RGB format and return the color string according to the value of the three channels passed to the
based on the value of the three channels passed to the function. The colors it can recognize are:
- red;
- yellow;
- green;
- blue;
- violet.
We made the necessary proportions to convert the parameters of the three channels since the colors went
in a range [0,1].

## const WbCameraRecognitionObject* is_that_color (const double *color)
This function allows to search, inside the list of the objects recognized by the camera, for the ball of the specified
color and returns its position in the list. If the color of the specified ball
is not present, the function returns a pointer to NULL.

## const double odomentry (const double Xf, const double Zf)
This function acquires the coordinates of the robot and calculates the distance to the goal to be reached. Finally,
returns the angle of rotation that the robot must make to move in the direction of the goal.

## void goal_oriented (const double theta_first)
This function takes the yaw angle of the robot, calculates the difference with
theta_first, passed as a parameter to the function and the result of the odometry function, and, finally, to rotate the
robot clockwise or counterclockwise until the difference between the two angles is negligible.
To optimize the direction of rotation, the robot will be rotated counterclockwise if the difference between the two angles is positive or in the opposite direction if the difference between the two angles is negligible.
is positive or in the opposite direction if the difference is negative, thus decreasing the time the robot
takes to orient itself towards the target.

## double distance_two_points (const double X0, const double Z0, const double Xf, const double Zf)
This function is used to calculate the distance between two points using Euclid distance. Are passed
the coordinates of the start and end point are passed to the function and it returns the distance between them.

## void avoid_obstacle (const double *color)
This function allows to perceive and avoid obstacles during the route by using distance sensors.
distance sensors. The return value of these sensors makes it possible to detect whether the obstacle is to the left or to the right of the robot.
of the robot by a boolean variable.

Based on the position of the obstacle, the robot makes movements that allow it to go around it and continue
the action it was performing previously.
If the robot, via sensor, perceives the ball inside the arms, it will be directed towards the
platform. If not, it will return to its initial state of searching for the ball.
