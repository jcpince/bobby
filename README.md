# Bobby the robot

Sources to operate a simple 2 wheels + caster wheel.

In order to simulate, simply run the following commands in two shells:
 - roslaunch bobby_gazebo bobby_world.launch
 - rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/bobby/cmd_vel

## NOTES:
 - Please refer to the ROS documentation to create a catkin workspace
 necessary to this project.
 - You might need to reset the world and pose in the Gazebo GUI prior
 to starting the stimulation.
