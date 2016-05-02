#include "Control.hpp"
#include <cmath>

void Control::pose_correction(double theta,double cheat_time)
{
  // ArRobot *robot;
  // robot = new ArRobot();
  double time_threshold = 80;
  double BASE_SPEED = 0.1, MOVE_TIME = 2, CLOCK_SPEED = 1;
  if (cheat_time>time_threshold)
  {
    BASE_SPEED = 0.1;
  }
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  rate.reset();
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
      // robot->lock();
      // robot->setVel(correct.linear.x*1000);
      // robot->setRotVel(correct.angular.z*180/M_PI);
      // robot->unlock();
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

void Control::follow_wall(int flag)
{
  double BASE_SPEED = 0.1, MOVE_TIME = 3, CLOCK_SPEED = 1;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  geometry_msgs::Twist follow_wall_first;
  geometry_msgs::Twist follow_wall_second;
  geometry_msgs::TwistStamped follow_wall_pub;
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
  ros::Publisher velocity =n.advertise<geometry_msgs::TwistStamped>("/control",1);
  if (flag==0)
  {
    follow_wall_first.angular.z = PI/10;
    follow_wall_second.angular.z = -PI/10;
  }
  else
  {
    follow_wall_first.angular.z = -PI/10;
    follow_wall_second.angular.z = PI/10;
  }
   while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
   {
    if (count == 0 || count == 1)
     {
      follow_wall_first.linear.x = BASE_SPEED; //publish the new velocity to rosaria
      // robot->lock();
      // robot->setVel(correct.linear.x*1000);
      // robot->setRotVel(correct.angular.z*180/M_PI);
      // robot->unlock();
      pub.publish(follow_wall_first);
      follow_wall_pub.twist=follow_wall_first;
      follow_wall_pub.twist.angular.z=follow_wall_pub.twist.angular.z/3;
     }
      count++;
      ROS_INFO_STREAM("The robot is now following wall!");
      follow_wall_pub.header.stamp.sec=ros::Time::now().toSec();
      velocity.publish(follow_wall_pub);
      ros::spinOnce();
      rate.sleep();
   }
  count=0;
   while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
   {
    if (count == 0 || count == 1)
     {
      follow_wall_second.linear.x = BASE_SPEED; //publish the new velocity to rosaria
      // robot->lock();
      // robot->setVel(correct.linear.x*1000);
      // robot->setRotVel(correct.angular.z*180/M_PI);
      // robot->unlock();
      pub.publish(follow_wall_second);
      follow_wall_pub.twist=follow_wall_second;
      follow_wall_pub.twist.angular.z=follow_wall_pub.twist.angular.z/3;
     }
      count++;
      ROS_INFO_STREAM("The robot is now following wall!");
      follow_wall_pub.header.stamp.sec=ros::Time::now().toSec();
      velocity.publish(follow_wall_pub);
      ros::spinOnce();
      rate.sleep();
   }
}