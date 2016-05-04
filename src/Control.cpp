#include "Control.hpp"
#include <cmath>

void Control::pose_correction(double theta,double cheat_time)
{
  // ArRobot *robot;
  // robot = new ArRobot();
  lock=0;
  double time_threshold = 80;
  double BASE_SPEED = 0.2, MOVE_TIME = 2, CLOCK_SPEED = 1;
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
  if (abs(0-theta)<threshold && !lock)
  {
      correct.angular.z=(0-theta);///int(MOVE_TIME/CLOCK_SPEED);
      lock=1;
  }
  if (abs(2*PI-theta)<threshold && !lock)
  {
      correct.angular.z=(2*PI-theta);///int(MOVE_TIME/CLOCK_SPEED);
      lock=1;
  }
  if (abs(PI/2-theta)<threshold && !lock)
  {
      correct.angular.z=(PI/2-theta);///int(MOVE_TIME/CLOCK_SPEED);
      lock=1;
  }
  if (abs(PI-theta)<threshold && !lock)
  {
      correct.angular.z=(PI-theta);///int(MOVE_TIME/CLOCK_SPEED);
      lock=1;
  }
  if (abs((3*PI/2-theta)<threshold) && !lock)
  {
       correct.angular.z=(3*PI/2-theta);///int(MOVE_TIME/CLOCK_SPEED);
       lock=1;
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
  double BASE_SPEED = 0.1, MOVE_TIME = 4, CLOCK_SPEED = 1;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  geometry_msgs::Twist follow_wall_first;
  geometry_msgs::Twist follow_wall_second;
  geometry_msgs::TwistStamped follow_wall_pub;
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
  ros::Publisher velocity =n.advertise<geometry_msgs::TwistStamped>("/control",1);
  if (flag==0)
  {
    follow_wall_first.angular.z = PI/20;
    follow_wall_second.angular.z = -PI/20;
  }
  else
  {
    follow_wall_first.angular.z = -PI/20;
    follow_wall_second.angular.z = PI/20;
  }
   while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
   {
    if (count == 0 || count == 1)
     {
      follow_wall_first.linear.x = BASE_SPEED; //publish the new velocity to rosaria
      pub.publish(follow_wall_first);
      follow_wall_pub.twist=follow_wall_first;
      follow_wall_pub.twist.angular.z=follow_wall_pub.twist.angular.z/3;
     }
      count++;
      ROS_INFO_STREAM("The robot is now avoiding wall!");
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
      pub.publish(follow_wall_second);
      follow_wall_pub.twist=follow_wall_second;
      follow_wall_pub.twist.angular.z=follow_wall_pub.twist.angular.z/3;
     }
      count++;
      ROS_INFO_STREAM("The robot is now avoiding wall!");
      follow_wall_pub.header.stamp.sec=ros::Time::now().toSec();
      velocity.publish(follow_wall_pub);
      ros::spinOnce();
      rate.sleep();
   }
}

bool Control::turn_left()
{
  ros::Publisher control = n.advertise<geometry_msgs::TwistStamped>("/control",1); 
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;
  geometry_msgs::TwistStamped msg_pub;
  double BASE_SPEED = 0.1, MOVE_TIME = 1, CLOCK_SPEED = 0.25; //0.05,1,0.25
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

  // stop before turning
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  msg_pub.twist=msg;
  pub.publish(msg);
  msg_pub.header.stamp.sec = ros::Time::now().toSec();
  control.publish(msg_pub);

  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED) //2
  {
      // Spin PI/2
      if (count == 0 || count == 1)
      {  
         msg.linear.x=BASE_SPEED;  
         msg.angular.z = PI/int(MOVE_TIME/CLOCK_SPEED) / 2;//2
         msg_pub.twist = msg;
         msg_pub.twist.angular.z= msg_pub.twist.angular.z/3;
         pub.publish(msg);
      }
      ROS_INFO_STREAM("The robot is now turning left!");
      msg_pub.header.stamp.sec = ros::Time::now().toSec();
      control.publish(msg_pub);
      count++;
      ros::spinOnce();
      rate.sleep();
  }    

  ROS_INFO_STREAM("The robot finished turning left for 90 degree!");
  
  return 0;
}

bool Control::turn_right()
{
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  ros::Publisher control=n.advertise<geometry_msgs::TwistStamped>("/control",1);
  geometry_msgs::Twist msg;
  geometry_msgs::TwistStamped msg_pub;
  double BASE_SPEED = 0.1, MOVE_TIME = 1, CLOCK_SPEED = 0.25; //1 && 0.25
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);


  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
  {
      // Spin PI/2
      if (count == 0 || count == 1)
      {
         msg.linear.x = BASE_SPEED;
         msg.angular.z = -1 * PI/int(MOVE_TIME/CLOCK_SPEED) / 2;
         msg_pub.twist=msg;
         msg_pub.twist.angular.z= msg_pub.twist.angular.z/3;
         pub.publish(msg);
      }
      ROS_INFO_STREAM("The robot is now turning right!");
      msg_pub.header.stamp.sec = ros::Time::now().toSec();
      control.publish(msg_pub);
      count++;
      ros::spinOnce();
      rate.sleep();
  }
  
  ROS_INFO_STREAM("The robot finished turning right for 90 degrees!");

  return 0;
}

bool Control::go_straight()
{
  ros::Publisher control=n.advertise<geometry_msgs::TwistStamped>("/control",1);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;
  geometry_msgs::TwistStamped msg_pub;
  double BASE_SPEED = 0.2, MOVE_TIME = 3, CLOCK_SPEED = 1;

  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
  {
      if (count == 0 || count == 1)
    {
      msg.linear.x = BASE_SPEED; //publish the new velocity to rosaria
      msg_pub.twist=msg;
      pub.publish(msg);

    }
      msg_pub.header.stamp.sec = ros::Time::now().toSec();
      control.publish(msg_pub);
      ROS_INFO_STREAM("The robot is now moving forward!");
      count++;
      ros::spinOnce();
      rate.sleep();
   }

   return 0;
}