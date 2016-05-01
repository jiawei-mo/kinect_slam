#include "Control_Node.hpp"

Control_Node::Control_Node()
{
  double ini_CLOCK_SPEED=1;
  double Initialization_time=6;
  double ini_count=0;
  // ros::Rate ini_rate(ini_CLOCK_SPEED);
  turn_time=ros::Time::now().toSec();
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
  sonar = nh.subscribe("RosAria/sonar",1, &Control_Node::sonarMeassageReceived,this);
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  char action;
  double current_time=ros::Time::now().toSec();
  if(msg.points[0].y>=LEFT_AVAILABLE && current_time-turn_time>30) 
  { 
    //myCtrl.turn_left();
    action=system("rosrun kinect_slam turn_left");
    action = system("rosrun kinect_slam go_straight");
   //myCtrl.go_straight();  
   turn_count++;
   correct_count++;
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
   //if none of the situations above is satisfied, robot keep going straight and run pose correction
   //myCtrl.go_straight();
   if(msg.points[0].y>(distance_maintain+correction_threshold)) //right drift
   {
       if (msg.points[0].y<=ALERT_DISTANCE)
       {
        action=system("rosrun kinect_slam pose_correction_left");
        action=system("rosrun kinect_slam pose_correction_left");
        action=system("rosrun kinect_slam pose_correction_left");
        action=system("rosrun kinect_slam pose_correction_right");
        action=system("rosrun kinect_slam pose_correction_right");
        action=system("rosrun kinect_slam pose_correction_right");
       }
       else
       {
        action=system("rosrun kinect_slam pose_correction_left");
        action=system("rosrun kinect_slam pose_correction_right");
       }
       //action = system("rosrun kinect_slam go_straight");
   }
   else if(msg.points[0].y<(distance_maintain-correction_threshold)) //left drift
   {
       if(abs(msg.points[6].y)<ALERT_DISTANCE)
       {
         action=system("rosrun kinect_slam pose_correction_right");
         action=system("rosrun kinect_slam pose_correction_right");
         action=system("rosrun kinect_slam pose_correction_right");
         action=system("rosrun kinect_slam pose_correction_left");
         action=system("rosrun kinect_slam pose_correction_left");
         action=system("rosrun kinect_slam pose_correction_left");
       }
       else
       {
        action=system("rosrun kinect_slam pose_correction_right");
        action=system("rosrun kinect_slam pose_correction_left");
       }      
       // action = system("rosrun kinect_slam go_straight");
      // action=system("rosrun kinect_slam spose_correction_left");
   }
   else
   {
        action = system("rosrun kinect_slam go_straight");
   }
}
