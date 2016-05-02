#include "Control.hpp"
#include <cmath>

void Control::pose_correction(double theta)
{
  double BASE_SPEED = 0.2, MOVE_TIME = 2, CLOCK_SPEED = 1;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  rate.reset();
  double PI=3.14159;
  double threshold=PI/20;
  geometry_msgs::Twist correct;
  geometry_msgs::TwistStamped correct_pub;
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
  ros::Publisher velocity =n.advertise<geometry_msgs::TwistStamped>("/control",1);
  if (abs(0-theta)<threshold)
  {
      correct.angular.z=(0-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(2*PI-theta)<threshold)
  {
      correct.angular.z=(2*PI-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(PI/2-theta)<threshold)
  {
      correct.angular.z=(PI/2-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(PI-theta)<threshold)
  {
      correct.angular.z=(PI-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs((3*PI/2-theta)<threshold))
  {
       correct.angular.z=(3*PI/2-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
   {
    if (count == 0 || count == 1)
     {
      correct.linear.x = BASE_SPEED; //publish the new velocity to rosaria
      pub.publish(correct);
      correct_pub.twist=correct;
      correct_pub.twist.angular.z=correct_pub.twist.angular.z/3;
     }
      count++;
      ROS_INFO_STREAM("The robot is now correcting pose!");
      correct_pub.header.stamp.sec=ros::Time::now().toSec();
      velocity.publish(correct_pub);
      ros::spinOnce();
      rate.sleep();
   }
}