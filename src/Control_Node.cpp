#include "Control_Node.hpp"

Control_Node::Control_Node()
{
  double ini_CLOCK_SPEED=1;
  double Initialization_time=6;
  double ini_count=0;
  double distance_maintain =0.8;
  // ros::Rate ini_rate(ini_CLOCK_SPEED);
  turn_time=ros::Time::now().toSec();
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
  action_lock=0;
  sonar = nh.subscribe("RosAria/sonar",1, &Control_Node::sonarMeassageReceived,this);
  pose_correct=nh.subscribe("/pose",1, &Control_Node::poseMeassageReceived,this);
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  char action;
  double current_time=ros::Time::now().toSec();
  if(msg.points[0].y>=LEFT_AVAILABLE && (current_time-turn_time>80 || first_turn)&& !action_lock && turn_count!=1)    //30s for turn_lock
  { 
    action_lock = 1; //locking the system to prevent operation conflict
    action_lock = myCtrl.turn_left();
    // action_lock = 1;
    // action_lock = myCtrl.go_straight();
    // action=system("rosrun kinect_slam turn_left");
    // action = system("rosrun kinect_slam go_straight");
    turn_count++;
    turn_time=current_time;
    first_turn = 0;
  }
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[2].x>OBSTACLE_SIDES &&msg.points[1].x>OBSTACLE_SIDES && !action_lock) //avoid obstacle left
  {
    action_lock = 1;
    action_lock = myCtrl.turn_left();  
    action_lock = 1;
    action_lock = myCtrl.turn_right(); 
   // action=system("rosrun kinect_slam turn_left");
   // action=system("rosrun kinect_slam turn_right");
    turn_count++; 
  } 
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[4].x>OBSTACLE_SIDES &&msg.points[5].x>OBSTACLE_SIDES && !action_lock) //avoid obstacle left
  {
    action_lock = 1;
    action_lock = myCtrl.turn_right();  
    action_lock = 1;
    action_lock = myCtrl.turn_left(); 
    // action=system("rosrun kinect_slam turn_right");
    // action=system("rosrun kinect_slam turn_left");
    turn_count++;
  }
  //follow wall
  current_time = ros::Time::now().toSec();
 if ((msg.points[0].y<=1 || msg.points[6].y>-1) && !action_lock)//&& turn_count>0 && current_time-turn_time>30)
 {
   action_lock = 1;
   if ((msg.points[0].y-distance_maintain)<=1)
    action_lock = myCtrl.follow_wall(1); //slightly turn right
   else
    action_lock = myCtrl.follow_wall(0); //slightly turn left
 }
 //pose correction using EKF estimation
   current_time=ros::Time::now().toSec();
   double cheat_time = current_time - turn_time;
   if (!action_lock)
   { 
     myCtrl.pose_correction(current_theta,cheat_time);
   }
}

void Control_Node::poseMeassageReceived(const kinect_slam::Pose2DMsg &msg)
{
  current_theta=msg.theta;
}