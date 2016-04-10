#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer_stop");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
  geometry_msgs::Twist msg;

  // Make the robot stop (robot perhaps has a speed already)
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  pub.publish(msg);
  ROS_INFO_STREAM("Stopping the robot!");
  ros::spinOnce();
}

