#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Turn_left");
  ros::NodeHandle nh;
  ros::NodeHandle n;

  ros::Publisher control = n.advertise<geometry_msgs::Twist>("Pioneer_control/velocity",1); 
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;

  double BASE_SPEED = 0.05, MOVE_TIME = 1, CLOCK_SPEED = 0.25, PI = 3.14159;
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
	       msg.angular.z = PI/ int(MOVE_TIME/CLOCK_SPEED) / 2;//2
	       pub.publish(msg);
               control.publish(msg);
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
      control.publish(msg);
  }
  ROS_INFO_STREAM("The robot finished turning left for 90 degree!");
    
  //Guard, make sure the robot stops
  rate.sleep();
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  pub.publish(msg);
  control.publish(msg);
}
