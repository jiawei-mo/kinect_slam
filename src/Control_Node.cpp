#include "Control_Node.hpp"

Control_Node::Control_Node()
{
  turn_count = 0;
  sonar = nh.subscribe("RosAria/sonar",10, &Control_Node::sonarMeassageReceived, this);
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
 if (msg.points[3].x>=OBSTACLE_FRONT && (msg.points[0].y<LEFT_AVAILABLE)) //no obstacle detected in the front and left not available
  {
   myCtrl.go_straight();
   myCtrl.pose_correction();  
  }
  else
 {
  if(msg.points[0].y>=LEFT_AVAILABLE) 
  { 
   myCtrl.turn_left();  
   myCtrl.go_straight();  
   turn_count++;
  }
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[2].x>OBSTACLE_SIDES &&msg.points[1].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   myCtrl.turn_left();  
   myCtrl.turn_right();  
  } 
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[4].x>OBSTACLE_SIDES &&msg.points[5].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   myCtrl.turn_right();  
   myCtrl.turn_left();  
  }
 }
}
