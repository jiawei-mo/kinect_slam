#include "Control_Node.hpp"

Control_Node::Control_Node()
{
  double ini_CLOCK_SPEED=1;
  double Initialization_time=6;
  double ini_count=0;
<<<<<<< HEAD
  double distance_maintain =0.8;
=======
>>>>>>> 7848c2a4f7a859934ccc927b2a578e8138c89a7b
  // ros::Rate ini_rate(ini_CLOCK_SPEED);
  turn_time=ros::Time::now().toSec();
  correct_time=ros::Time::now().toSec();
  /*while (ros::ok() && ini_count<Initialization_time)
  {
    ini_count++;
    ini_rate.sleep();
  }
  */
  turn_count = 0;
  correct_count=0;
  distance_maintain=0.8;
  correction_threshold=0.1;
<<<<<<< HEAD
  follow_wall_count=0;
=======
>>>>>>> 7848c2a4f7a859934ccc927b2a578e8138c89a7b
  sonar = nh.subscribe("RosAria/sonar",1, &Control_Node::sonarMeassageReceived,this);
  pose_correct=nh.subscribe("/pose",1, &Control_Node::poseMeassageReceived,this);
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  char action;
  double current_time=ros::Time::now().toSec();
<<<<<<< HEAD
  if(msg.points[0].y>=LEFT_AVAILABLE && current_time-turn_time>20) 
=======
  if(msg.points[0].y>=LEFT_AVAILABLE && current_time-turn_time>0) 
>>>>>>> 7848c2a4f7a859934ccc927b2a578e8138c89a7b
  { 
    //myCtrl.turn_left();
    // double stop_time=2;
    // double stop_speed=1;
    // ros::Rate stop_rate(stop_speed);
    // int stop_count=0;
    // geometry_msgs::TwistStamped stop_velocity_pub;
    // geometry_msgs::Twist stop_velocity;
    // ros::Publisher stop_pub = nh.advertise<geometry_msgs::TwistStamped>("/control",1);
    // ros::Publisher stop_ctrl = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    // while(ros::ok() && stop_count<stop_time/stop_speed)
    // {
    //  stop_velocity.linear.x=0;
    //  stop_ctrl.publish(stop_velocity);
    //  stop_velocity_pub.twist=stop_velocity;
    //  stop_velocity_pub.header.stamp.sec=ros::Time::now().toSec();
    //  stop_count++;
    //  stop_rate.sleep();
    // }
    action=system("rosrun kinect_slam turn_left");
    action = system("rosrun kinect_slam go_straight");
<<<<<<< HEAD
    pre_follow_wall_time = ros::Time::now().toSec();
   //myCtrl.go_straight();  
   turn_count++;
   correct_count++;
   follow_wall_count++;
=======
   //myCtrl.go_straight();  
   turn_count++;
   correct_count++;
>>>>>>> 7848c2a4f7a859934ccc927b2a578e8138c89a7b
   turn_time=current_time;
  }
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[2].x>OBSTACLE_SIDES &&msg.points[1].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   // myCtrl.turn_left();  
   //myCtrl.turn_right(); 
   action=system("rosrun kinect_slam turn_left");
   action=system("rosrun kinect_slam turn_right");
   correct_count++; 
  } 
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[4].x>OBSTACLE_SIDES &&msg.points[5].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   //myCtrl.turn_right();  
   //myCtrl.turn_left();  
    action=system("rosrun kinect_slam turn_right");
    action=system("rosrun kinect_slam turn_left");
    correct_count++;
  }
<<<<<<< HEAD
  //follow wall
  current_time = ros::Time::now().toSec();
 if ((msg.points[0].y-distance_maintain)>0.4 && turn_count>0 && follow_wall_count==0)
 {
   action=system("rosrun kinect_slam pose_correction_left");
   action=system("rosrun kinect_slam pose_correction_right");
   turn_count=0;
   follow_wall_count=0;
 }
 //pose correction using EKF estimation
  double correct_current_time=ros::Time::now().toSec();
    // if (correct_count>0 || correct_time-correct_current_time>15)
    // {
  myCtrl.pose_correction(current_theta);
     //correct_count=0;
    //correct_time=correct_current_time;
    // }
    ///////////////////////////////////////////////////
    //action = system("rosrun kinect_slam go_straight");
=======
//pose correction using EKF estimation
  double correct_current_time=ros::Time::now().toSec();
    if (correct_count>0 || correct_time-correct_current_time>15)
    {
      myCtrl.pose_correction(current_theta);
      correct_count=0;
      correct_time=correct_current_time;
    }
    ///////////////////////////////////////////////////
    action = system("rosrun kinect_slam go_straight");
>>>>>>> 7848c2a4f7a859934ccc927b2a578e8138c89a7b
}

void Control_Node::poseMeassageReceived(const geometry_msgs::Pose2D &msg)
{
  current_theta=msg.theta;
}