#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_ahead");
  ros::NodeHandle nh;
  ros::NodeHandle n;
  
  ros::Publisher control=n.advertise<geometry_msgs::Twist>("Pioneer_control/velocity",1);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;

  double BASE_SPEED = 0.1, MOVE_TIME = 1.0, CLOCK_SPEED = 0.5;
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
	    pub.publish(msg);
            control.publish(msg);
	}
      ROS_INFO_STREAM("The robot is now moving forward!");
      count++;
      ros::spinOnce();
      rate.sleep();
   }
  }
