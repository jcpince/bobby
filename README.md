# Bobby the robot

Sources to operate a simple 2 wheels + caster wheel.

In order to simulate, simply run the following commands in two shells:
 - roslaunch bobby_gazebo bobby_world.launch
 - rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/bobby/cmd_vel

In order to vizualize with RViz:
 - Launch the world as described before
 - rosrun rviz rviz
   - In the GUI, change the Global Options->Fixed Frame to odom
   - Add the components RobotModel and TF
   - Add the component /bobby/camera1/image_raw/Image by topic

In order to preview the image:
 - Launch the world as described before
 - rosrun image_view image_view image:=/bobby/camera1/image_raw


## NOTES:
 - Please refer to the ROS documentation to create a catkin workspace
 necessary to this project.
 - You might need to reset the world and pose in the Gazebo GUI prior
 to starting the stimulation.
