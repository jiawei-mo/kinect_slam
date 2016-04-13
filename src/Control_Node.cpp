#include "Control_Node.hpp"

Control_Node::Control_Node()
{
  turn_count = 0;
  sonar = nh.subscribe("RosAria/sonar",1, &Control_Node::sonarMeassageReceived, this);
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  double CLOCK_SPEED=3;
  ros::Rate rate(CLOCK_SPEED);
  char action;
  if(msg.points[0].y>=LEFT_AVAILABLE) 
  { 
    //myCtrl.turn_left();
    action=system("rosrun kinect_slam turn_left");
   //myCtrl.go_straight();  
   turn_count++;
  }
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[2].x>OBSTACLE_SIDES &&msg.points[1].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   // myCtrl.turn_left();  
   //myCtrl.turn_right(); 
   action=system("rosrun kinect_slam turn_left");
   action=system("rosrun kinect_slam turn_right"); 
  } 
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[4].x>OBSTACLE_SIDES &&msg.points[5].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   //myCtrl.turn_right();  
   //myCtrl.turn_left();  
    action=system("rosrun kinect_slam turn_right");
    action=system("rosrun kinect_slam turn_left");
  }
   //if none of the situations above is satisfied, robot keep going straight and run pose correction
   myCtrl.go_straight();
   //TODO
   //myCtrl.pose_correction();
   rate.sleep();
}
