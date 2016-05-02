#include<ros/ros.h>
#include<geometry_msgs/TwistStamped.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Time.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_straight");
  ros::NodeHandle nh;
  
  ros::Publisher control=nh.advertise<geometry_msgs::TwistStamped>("/control",1);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
 // ros::Publisher _time = nh.advertise<std_msgs::Time>("/current_time",1);
  geometry_msgs::Twist msg;
  geometry_msgs::TwistStamped msg_pub;
 // std_msgs::Time current;
<<<<<<< HEAD
  double BASE_SPEED = 0.2, MOVE_TIME = 2, CLOCK_SPEED = 1;
=======
  double BASE_SPEED = 0.1, MOVE_TIME = 4, CLOCK_SPEED = 1;
>>>>>>> 7848c2a4f7a859934ccc927b2a578e8138c89a7b
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

  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
  {
      if (count == 0 || count == 1)
	  {
	    msg.linear.x = BASE_SPEED; //publish the new velocity to rosaria
      msg_pub.twist=msg;
      //current.sec=ros::Time::now().toSec();
	    pub.publish(msg);
     // _time.publish(current);

	  }
      msg_pub.header.stamp.sec = ros::Time::now().toSec();
      control.publish(msg_pub);
      ROS_INFO_STREAM("The robot is now moving forward!");
      count++;
      ros::spinOnce();
      rate.sleep();
   }
  }
