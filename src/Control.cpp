#include "Control.hpp"

void Control::go_straight()
{
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;

  double BASE_SPEED = 0.1, MOVE_TIME = 1.0, CLOCK_SPEED = 0.5;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

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

  double BASE_SPEED = 0.05, MOVE_TIME = 1, CLOCK_SPEED = 0.25, PI = 3.14159;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

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