# rainbow

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
