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
  follow_wall_time = ros::Time::now();
  action_lock=0;
  sonar = nh.subscribe("RosAria/sonar",1, &Control_Node::sonarMeassageReceived,this);
  pose_correct=nh.subscribe("/pose",1, &Control_Node::poseMeassageReceived,this);
  pose_corrected = 0;
  current_theta = 0;
  turn_flag=0;
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  char action;
  current_time=ros::Time::now();
  pose_corrected = myCtrl.check_pose(current_theta);
  //std::cout<<"check_pose is " << pose_corrected<<std::endl;
  if(msg.points[0].y>=LEFT_AVAILABLE && (current_time.sec-turn_time.sec>50)&& (current_time.sec-follow_wall_time.sec>10) &&!action_lock && pose_corrected)    //30s for turn_lock
  { 
    action_lock = 1; //locking the system to prevent operation conflict
    action_lock = myCtrl.turn_left();
    // action_lock = 1;
    // action_lock = myCtrl.go_straight();
    // action=system("rosrun kinect_slam turn_left");
    // action = system("rosrun kinect_slam go_straight");
    turn_count++;
    turn_flag =2;
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
  current_time = ros::Time::now();
 if ((msg.points[0].y<=0.7 || msg.points[6].y>-0.9) && !action_lock && ((current_time.sec-follow_wall_time.sec>20 && current_time.sec-turn_time.sec>20 )|| first_turn))//&& turn_count>0 && current_time-turn_time>30)
 {
   follow_wall_time = ros::Time::now();
   action_lock = 1;
   if ((msg.points[0].y<=0.7) && (msg.points[6].y>-0.9))
   {

   }
   else if (msg.points[0].y<=0.7)
   {
    action_lock = myCtrl.follow_wall(1,1,msg.points[0].y); //slightly turn right
    action_lock = 1;
    action_lock = myCtrl.follow_wall(0,2,msg.points[0].y);
    turn_flag = 2;
   }
   else
   {
    action_lock = myCtrl.follow_wall(0,1,msg.points[6].y); //slightly turn left
    action_lock =1;
    action_lock = myCtrl.follow_wall(1,2,msg.points[6].y);
    turn_flag = 2;
   }
}
 //pose correction using EKF estimation
   current_time=ros::Time::now();
   double cheat_time = current_time.sec - turn_time.sec;
   if (!action_lock)
   { 
     action_lock = 1;
     action_lock = myCtrl.pose_correction(current_theta,turn_flag);
     turn_flag--;
   }
}

void Control_Node::poseMeassageReceived(const geometry_msgs::PoseStamped &msg)
{
  double temp_theta_z = msg.pose.orientation.z;
  double temp_theta_w = msg.pose.orientation.w;
  current_theta=atan2(temp_theta_z,temp_theta_w)*2;
}