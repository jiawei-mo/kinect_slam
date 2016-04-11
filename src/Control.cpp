#include "Control.hpp"
#include <cmath>
void Control::go_straight()
{
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;

  double BASE_SPEED = 0.1, MOVE_TIME = 1.0, CLOCK_SPEED = 0.5;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  rate.reset();
  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
  {
      if (count == 0 || count == 1)
	  {
	    msg.linear.x = BASE_SPEED; //publish the new velocity to rosaria
	    pub.publish(msg);
	  }
      ROS_INFO_STREAM("The robot is now moving forward!");
      count++;
      ros::spinOnce();
      rate.sleep();
  }
}

void Control::turn_left()
{
  ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;

  double BASE_SPEED = 0.05, MOVE_TIME = 1.0, CLOCK_SPEED = 0.25, PI = 3.14159;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  rate.reset();
  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED) //2
  {
      // Spin PI/2
      if (count == 0 || count == 1)
	    {
        msg.linear.x=BASE_SPEED;  
	      msg.angular.z = PI/ int(MOVE_TIME/CLOCK_SPEED) / 2;//2
	      pub.publish(msg);
	    }
      ROS_INFO_STREAM("The robot is now turning left!");
      count++;
      ros::spinOnce();
      rate.sleep();
  }    

    // Stop the spin
  for(int i = 0; i < 2; i++)
  {
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = 0;
      pub.publish(msg);
  }
  ROS_INFO_STREAM("The robot finished turning left for 90 degree!");
    
  //Guard, make sure the robot stops
  rate.sleep();
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  pub.publish(msg);
}

void Control::turn_right()
{
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;

  double BASE_SPEED = 0.05, MOVE_TIME = 1, CLOCK_SPEED = 0.25, PI = 3.14159;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  rate.reset();
  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
  {
      // Spin PI/2
      if (count == 0 || count == 1)
	    {
               msg.linear.x = BASE_SPEED;
	       msg.angular.z = -1 * PI/ int(MOVE_TIME/CLOCK_SPEED) / 2;
	       pub.publish(msg);
	    }
      ROS_INFO_STREAM("The robot is now turning right!");
      count++;
      ros::spinOnce();
      rate.sleep();
  }

    // Stop the spin
  for(int i = 0; i < 2; i++)
  {
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = 0;
      pub.publish(msg);
  }
  ROS_INFO_STREAM("The robot finished turning right for 90 degrees!");
}

void Control::pose_correction()
{  
  pose_correct=n.subscribe("kinect_slam/pose",1,&Control::poseMeassageReceived,this);
}
void Control::poseMeassageReceived(const geometry_msgs::Twist &msg)
{
  double BASE_SPEED = 0.1, MOVE_TIME = 1.0, CLOCK_SPEED = 0.5;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  rate.reset();
  double PI=3.14159;
  double threshold=PI/20;
  geometry_msgs::Twist correct;
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
  if (abs(0-msg.angular.z)<threshold)
  {
     if (msg.angular.z>=0)
      correct.angular.z=(0-msg.angular.z)/int(MOVE_TIME/CLOCK_SPEED);
    else
      correct.angular.z=(msg.angular.z)/int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(PI/2-msg.angular.z)<threshold)
  {
    if (msg.angular.z<=PI/2)
      correct.angular.z=(PI/2-msg.angular.z)/int(MOVE_TIME/CLOCK_SPEED);
    else
      correct.angular.z=(-PI/2+msg.angular.z)/int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(PI-abs(msg.angular.z))<threshold)
  {
    if (msg.angular.z>=0)
      correct.angular.z=(PI-msg.angular.z)/int(MOVE_TIME/CLOCK_SPEED);
    else
      correct.angular.z=(-PI+abs(msg.angular.z))/int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(-PI/2-msg.angular.z)<threshold)
  {
       correct.angular.z=(-PI/2+abs(msg.angular.z))/int(MOVE_TIME/CLOCK_SPEED);
  }
  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
   {
    if (count == 0 || count == 1)
     {
      correct.linear.x = BASE_SPEED; //publish the new velocity to rosaria
      pub.publish(correct);
     }
      count++;
      ros::spinOnce();
      rate.sleep();
   }
}