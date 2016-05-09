#include "Control.hpp"
#include <cmath>

bool Control::pose_correction(double theta, int turn_flag, int turn_count)
{
  // ArRobot *robot;
  // robot = new ArRobot();
  //double time_threshold = 80;
  lock = 0;
  double BASE_SPEED = 0.4, MOVE_TIME = 2, CLOCK_SPEED = 1;
  if (turn_flag>0)
  {
    BASE_SPEED = 0.05;
  }
  int count = 0;
  double threshold = PI/10;
  ros::Rate rate(CLOCK_SPEED);
  geometry_msgs::Twist correct;
  geometry_msgs::TwistStamped correct_pub;
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
  ros::Publisher velocity =n.advertise<geometry_msgs::TwistStamped>("/control",1);
  // ros::Publisher test_theta = n.advertise<geometry_msgs::Pose2D>("/test_theta",1);
  // geometry_msgs::Pose2D temp_theta;
  // temp_theta.theta=theta;
  ///////////////////
  // double desire_orientation;
  // switch (turn_count)
  // {
  //   case 0 : desire_orientation = 0;
  //            break;
  //   case 1: desire_orientation = PI/2;
  //            break;
  //   case 2: desire_orientation = PI;
  //            break;
  //   case 3 : desire_orientation = 3*PI/2;
  //            break;
  //   default : desire_orientation = PI*2;
  //            break;
  // }
  // correct.angular.z = desire_orientation - theta;
  // if (desire_orientation == 0 && theta > PI*3/2)
  // {
  //    correct.angular.z = 2*PI - theta;
  // }
  ///////////
  // double desire_orientation = 0;
  // double matching_distance;
  // double min_matching_distance = 3*PI; 
  // for (int i=0;i<4;i++)
  //  {
  //     matching_distance = abs(desire_orientation-i*PI/2);
  //     if(matching_distance<min_matching_distance)
  //     {
  //       desire_orientation=i*PI/2;
  //       min_matching_distance=matching_distance;
  //     }
  //  }
  //  if(abs(2*PI-theta)<threshold)
  //  desire_orientation = 2*PI; 
  //  correct.angular.z = desire_orientation - theta;
   ////////////
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
      correct_pub.header.stamp.sec = ros::Time::now().sec;
      correct_pub.header.stamp.nsec =ros::Time::now().nsec;
      velocity.publish(correct_pub);
      // test_theta.publish(temp_theta);
      ros::spinOnce();
      rate.sleep();
   }

   return 0;

}

bool Control::follow_wall(int flag, int step_flag, double distance)
{
  double MOVE_TIME = 3, CLOCK_SPEED = 1;       //3,1
  double BASE_SPEED=0.6;
  double maintain_distance = distance>=0? 1.2:1.2; 
  current_sonar = n.subscribe("RosAria/sonar",1, &Control::sonarMeassageReceived,this);
  ////////normal control///////
  if(step_flag==2)
  {
  	BASE_SPEED = 0.05;
  }
  // else
  // {
  // 	BASE_SPEED = 0.1;
  // 	if((distance<0 && distance >-0.5)||(distance>0 && distance <0.5))
  // 	{
  // 		MOVE_TIME = 2;
  // 	}
  // }
  ///////PID control/////
  double K_p = 2;
  double K_i = 0.1;
  double K_d = 0.5;
  double PID_factor;
  double prop = distance>=0 ? maintain_distance-current_left : maintain_distance+current_right; 
  double integ = 0;
  double deriv;
  double current_error;
  double pre_error = prop;
  //////////////////
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);
  geometry_msgs::Twist follow_wall_first;
  geometry_msgs::TwistStamped follow_wall_pub;
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
  ros::Publisher velocity =n.advertise<geometry_msgs::TwistStamped>("/control",1);
  if (flag==0)
  {
    follow_wall_first.angular.z = PI/8;
    // follow_wall_second.angular.z = -PI/20;  //15
  }
  else
  {
    follow_wall_first.angular.z = -PI/8;
    // follow_wall_second.angular.z = PI/15;
    // follow_wall_rect.angular.z = PI/20;
  }
   PID_factor = 0.3;
   while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
   {
      //compute PID gain
        current_error = distance>=0 ? maintain_distance - current_left : maintain_distance + current_right;
        integ += current_error;
        deriv = pre_error-current_error;
        if (deriv<=0.5 && prop>=0 && prop<2)
        {
   	     PID_factor = K_p*prop + K_i*integ + K_d*deriv;
   	    }
   	    std::cout<<"current PID factor is " <<PID_factor<<std::endl;
      //
      follow_wall_first.linear.x = BASE_SPEED*PID_factor;
      follow_wall_first.angular.z = follow_wall_first.angular.z;//* PID_factor;
      pub.publish(follow_wall_first);
      follow_wall_pub.twist=follow_wall_first;
      follow_wall_pub.twist.angular.z=follow_wall_pub.twist.angular.z/3;
      count++;
      ROS_INFO_STREAM("The robot is now avoiding wall!");
      follow_wall_pub.header.stamp.sec=ros::Time::now().sec;
      follow_wall_pub.header.stamp.nsec=ros::Time::now().nsec;
      velocity.publish(follow_wall_pub);
      ros::spinOnce();
      rate.sleep();
   }

   ROS_INFO_STREAM("The robot finished avoiding wall!");
   return 0;
}

bool Control::turn_left()
{
  ros::Publisher control = n.advertise<geometry_msgs::TwistStamped>("/control",1); 
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;
  geometry_msgs::TwistStamped msg_pub;
  double BASE_SPEED = 0.2, MOVE_TIME = 4, CLOCK_SPEED = 1; //0.05,1,0.25
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
  msg_pub.header.stamp = ros::Time::now();
  control.publish(msg_pub);

  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED) //2
  {
      // Spin PI/2
      if (count == 0 || count == 1)
      {  
         msg.linear.x=BASE_SPEED;  
         msg.angular.z = PI/2;//int(MOVE_TIME/CLOCK_SPEED) / 2;//2
         msg_pub.twist = msg;
         msg_pub.twist.angular.z= msg_pub.twist.angular.z/3;
         pub.publish(msg);
      }
      ROS_INFO_STREAM("The robot is now turning left!");
      msg_pub.header.stamp.sec = ros::Time::now().sec;
      msg_pub.header.stamp.nsec = ros::Time::now().nsec;
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
  double BASE_SPEED = 0.2, MOVE_TIME = 4, CLOCK_SPEED = 1; //1 && 0.25
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);


  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED)
  {
      // Spin PI/2
      if (count == 0 || count == 1)
      {
         msg.linear.x = BASE_SPEED;
         msg.angular.z = -1 * PI/2;//int(MOVE_TIME/CLOCK_SPEED) / 2;
         msg_pub.twist=msg;
         msg_pub.twist.angular.z= msg_pub.twist.angular.z/3;
         pub.publish(msg);
      }
      ROS_INFO_STREAM("The robot is now turning right!");
      msg_pub.header.stamp.sec = ros::Time::now().sec;
      msg_pub.header.stamp.nsec = ros::Time::now().nsec;
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
  double BASE_SPEED = 0.3, MOVE_TIME = 3, CLOCK_SPEED = 1;

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
      msg_pub.header.stamp.sec = ros::Time::now().sec;
      msg_pub.header.stamp.nsec = ros::Time::now().nsec;
      control.publish(msg_pub);
      ROS_INFO_STREAM("The robot is now moving forward!");
      count++;
      ros::spinOnce();
      rate.sleep();
   }

   return 0;
}

bool Control::check_pose(double theta)
{
	double threshold = PI/90;
	if ( abs(theta-0)<threshold || abs(theta-PI/2)<threshold || abs(theta - PI) <threshold || abs(theta - 3*PI/2)<threshold || abs(2*PI-theta)<threshold)
	{
		return 1;
	}
	else
		return 0;
}

 double Control::compute_pose_correct(double theta)
{
  double threshold = PI/20;
  if (abs(0-theta)<threshold)
  {
      return (0-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(2*PI-theta)<threshold)
  {
      return (2*PI-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(PI/2-theta)<threshold)
  {
      return (PI/2-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(PI-theta)<threshold)
  {
      return (PI-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
  if (abs(3*PI/2-theta)<threshold)
  {
       return (3*PI/2-theta);///int(MOVE_TIME/CLOCK_SPEED);
  }
}

void Control::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
 {
   current_left = msg.points[0].y;
   current_right = msg.points[6].y;
 }
