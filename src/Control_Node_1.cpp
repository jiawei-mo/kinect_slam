#include "Control_Node.hpp"

Control_Node::Control_Node()
{
  double ini_CLOCK_SPEED=1;
  double Initialization_time=6;
  double ini_count=0;
  double distance_maintain =0.8;
  // ros::Rate ini_rate(ini_CLOCK_SPEED);
  turn_time=ros::Time::now();
  /*while (ros::ok() && ini_count<Initialization_time)
  {
    ini_count++;
    ini_rate.sleep();
  }
  */
  first_turn = 1;
  turn_count = 0;
  distance_maintain=0.8;
  correction_threshold=0.1;
  follow_wall_time = 0;
  action_lock=0;
  pub=nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
  velocity = nh.advertise<geometry_msgs::TwistStamped>("/control_repub",1);
  sonar = nh.subscribe("RosAria/sonar",1, &Control_Node::sonarMeassageReceived,this);
  pose_correct=nh.subscribe("/pose",1, &Control_Node::poseMeassageReceived,this);
  pose_corrected = 1;
  current_theta = 0;
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  geometry_msgs::Twist correct;
  geometry_msgs::TwistStamped correct_pub;
  double theta;
  double BASE_SPEED =0.2;
  if (turn_count<4 && ros::ok())
  {
    current_time = ros::Time::now();
  if(msg.points[0].y>=LEFT_AVAILABLE && (current_time.sec-turn_time.sec>30)&& !action_lock)    //30s for turn_lock
   { 
    action_lock = 1; //locking the system to prevent operation conflict
    action_lock = myCtrl.turn_left();
    turn_count++;
    turn_time=current_time;
    first_turn = 0;
   }
  else
   {
   if(msg.points[0].y<=1)
    {
      avoid_wall= -PI/20;
      ROS_INFO_STREAM("The robot is now correcting pose!");
    } 
    else if (msg.points[6].y>-1)
    {
      avoid_wall = PI/20;
      ROS_INFO_STREAM("The robot is now correcting pose!");
    }
    else
    {
        avoid_wall = 0;
        ROS_INFO_STREAM("The robot is now moving forward!");
    }
        theta = myCtrl.compute_pose_correct(current_theta);
        theta = theta + avoid_wall;
        correct.linear.x = BASE_SPEED;
        correct.angular.z = theta;
        correct_pub.twist = correct;
        correct_pub.header.stamp.sec = current_time.sec;
        correct_pub.header.stamp.nsec = current_time.nsec;
        velocity.publish(correct_pub);
        pub.publish(correct);
    }
  }
  else
  {
        correct.linear.x = 0;
        correct.angular.z = 0;
        correct_pub.twist = correct;
        correct_pub.header.stamp.sec = current_time.sec;
        correct_pub.header.stamp.nsec = current_time.nsec;
        pub.publish(correct);
        velocity.publish(correct_pub);
  }
  ros::spinOnce();
}

void Control_Node::poseMeassageReceived(const geometry_msgs::PoseStamped &msg)
{
  double temp_theta_z = msg.pose.orientation.z;
  double temp_theta_w = msg.pose.orientation.w;
  current_theta=atan2(temp_theta_z,temp_theta_w)*2;
}