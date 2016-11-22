#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "BobbyHW.hpp"

BobbyHW *bobby_hw = NULL;

void robotCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Linear speed: [%f, %f, %f]", msg->linear.x, msg->linear.y, msg->linear.z);
  ROS_INFO("Angular heading: [%f, %f, %f]", msg->angular.x, msg->angular.y, msg->angular.z);
  double linear_velocity = msg->linear.x, angular_velocity = msg->angular.z;
  bobby_hw->update(linear_velocity, angular_velocity);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bobby_robot_control");
  ros::NodeHandle node;

  bobby_hw = new BobbyHW(node, 0.2f, 0.4f, 100);

  ros::Subscriber sub = node.subscribe("robot_cmd", 1000, robotCmdCallback);

  ros::spin();

  delete bobby_hw;
  return 0;
}
