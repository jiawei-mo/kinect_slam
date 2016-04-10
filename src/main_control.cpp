#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <iomanip>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <rosaria/BumperState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#define Quit 0X71
#define KEYCODE_1 0x31
#define OBSTACLE_FRONT 2
#define OBSTACLE_SIDES 3
#define LEFT_AVAILABLE 3
#define LEFT_AVAILABLE_SIDES 4
static struct termios new1, old;
ros::Subscriber* sonar;
int turn_count;
/* initializes the terminal to new input settings */
void initTermios(int echo) 
{
    tcgetattr(0, &old); /* grab old terminal i/o settings */
    new1 = old; /* make new settings same as old settings */
    new1.c_lflag &= ~ICANON; /* disable buffered i/o */
    new1.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
    tcsetattr(0, TCSANOW, &new1); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios() 
{
    tcsetattr(0, TCSANOW, &old);
}

/* quit the program cleanly and close ros */
void quit()
{
        char action;
        action=system("rosrun kinect_slam pioneer_stop");
        resetTermios(); /* reset the terminal to old settings */
	ros::shutdown(); /*shutdown ros */
	exit(1); /* exit the system */
}

void encoderMessageReceived(const geometry_msgs::Twist& msg) 
{
	ros::NodeHandle n;
        ros::Publisher encoder_pub = n.advertise<geometry_msgs::Twist>("pioneer_control/wheel", 1);
        geometry_msgs::Twist encoder;
        double wheel_gap=0.1;
        double linear=msg.linear.x;
        double angular=msg.angular.z;
        encoder.linear.x=(2*linear+wheel_gap*angular)/2;
        encoder.linear.y=(2*linear-wheel_gap*angular)/2;
        encoder_pub.publish(encoder);
        std::cout<<"1";
        ros::spin();

}
void bumperStateMessageReceived(const rosaria::BumperState &msg)
{
    int front_size, rear_size;
    char action;
    int bumper_msg_count = 0;   
    if (bumper_msg_count == 0)
    {
    	front_size = sizeof(msg.front_bumpers) / sizeof(bool);
    	rear_size = sizeof(msg.rear_bumpers) / sizeof(bool);
    	for (int i=0;i<front_size;i++)
        {
	  if (msg.front_bumpers[i])
          {    
    	  std::cout<<"no obstacle in the front"<<std::endl;
          //action=system("rosrun pioneer_control go_ahead");
          }
	  else
          {
           std::cout<<"obstacle detected"<<std::endl;
           //action=system("rosrun pioneer_control go_ahead"); //undone
          }          
    	std::cout<<std::endl;
    	bumper_msg_count++;
        }
    }
}
void sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  if (turn_count==4)
    quit();
  //make decision based on point cloud returned from sonars
 char action;
 std::cout<<"testsonarinfo"<<","<<msg.points[1].x<<"\n"<<std::endl;
 int test=0;  //testing each action, set test=0 for running the whole program 
 if(test==1)
 {
   action=system("rosrun kinect_slam turn_left");
   action=system("rosrun kinect_slam go_ahead");
   action=system("rosrun kinect_slam turn_right");

 }
 else
{
 if (msg.points[3].x>=OBSTACLE_FRONT && (msg.points[0].y<LEFT_AVAILABLE)) //no obstacle detected in the front and left not available
  {
   action=system("rosrun kinect_slam go_ahead");  //activating new rosnode for actions
   //action=system("rosnode kill /go_ahead");
  }
  else
 {
  if(msg.points[0].y>=LEFT_AVAILABLE) //left available
  { 
   action=system("rosrun kinect_slam turn_left");
   action=system("rosrun kinect_slam go_ahead");
   turn_count++;
  }
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[2].x>OBSTACLE_SIDES &&msg.points[1].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   action=system("rosrun kinect_slam turn_left");
  // action=system("rosrun pioneer_control go_ahead");
   action=system("rosrun kinect_slam turn_right");
  } 
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[4].x>OBSTACLE_SIDES &&msg.points[5].x>OBSTACLE_SIDES) //avoid obstacle left
  {
   action=system("rosrun kinect_slam turn_right");
  // action=system("rosrun pioneer_control go_ahead");
   action=system("rosrun kinect_slam turn_left");
  }
 }
 }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Main_control");
  ros::NodeHandle nh;
  double Execute_TIME = 10.0, CLOCK_SPEED = 0.5;
  ros::Rate rate(CLOCK_SPEED);
  ros::Subscriber bumper_state, wheel_encoder;
  sonar = new ros::Subscriber;
  turn_count=0;
  initTermios(0);
  char action;
  int count=0;

while (ros::ok()&&count<=Execute_TIME/CLOCK_SPEED)
{
   std::cout << "Press num_1 to start, or hit q to quit: "<< std::endl;  // quit unfinished, the robot still can not stop
   std::cin >> action;	
  switch(action)
{
 case KEYCODE_1:
 {
  wheel_encoder=nh.subscribe("RosAria/cmd_vel", 1, &encoderMessageReceived); 
  *sonar= nh.subscribe("RosAria/sonar",10, &sonarMeassageReceived);                 //control based on sonar with buffer space 10
  //bumper_state = nh.subscribe("RosAria/bumper_state", 1000, &bumperStateMessageReceived) ;  //front obstacle avoidance  
 }
 break;
 case Quit:
 {
  quit();
  return false;
 }
}
 ros::spin();
 count++;
}
return 1;
}
