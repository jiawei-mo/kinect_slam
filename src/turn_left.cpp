#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Time.h>
#include <stdio.h>
#include <cmath>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Turn_left");
  ros::NodeHandle nh;
  // ArRobot *robot;
  // robot = new ArRobot();
  ros::Publisher control = nh.advertise<geometry_msgs::TwistStamped>("/control",1); 
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  //ros::Publisher _time = nh.advertise<std_msgs::Time>("/current_time",1);
  geometry_msgs::Twist msg;
  geometry_msgs::TwistStamped msg_pub;
  //std_msgs::Time current;
  double BASE_SPEED = 0.1, MOVE_TIME = 1, CLOCK_SPEED = 0.25, PI = 3.14159; //0.05,1,0.25
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

  // Make the robot stop (robot perhaps has a speed already) 
  /*msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  pub.publish(msg);*/

  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED) //2
  {
      // Spin PI/2
      if (count == 0 || count == 1)
	    {  
         msg.linear.x=BASE_SPEED;  
	       msg.angular.z = PI/int(MOVE_TIME/CLOCK_SPEED) / 2;//2
         msg_pub.twist = msg;
         msg_pub.twist.angular.z= msg_pub.twist.angular.z/3;
         //current.sec = ros::Time::now().toSec();
	      // robot->lock();
       //  robot->setVel(msg.linear.x*1000);
       //  robot->setRotVel(msg.angular.z*180/M_PI);
       //  robot->unlock();
         pub.publish(msg);
         //_time.publish(current);
	    }
      ROS_INFO_STREAM("The robot is now turning left!");
      msg_pub.header.stamp.sec = ros::Time::now().toSec();
      control.publish(msg_pub);
      count++;
      ros::spinOnce();
      rate.sleep();
  }    

  ROS_INFO_STREAM("The robot finished turning left for 90 degree!");
    
}
