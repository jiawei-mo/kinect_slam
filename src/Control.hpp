#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

class Control
{
public:
	void go_straight();
	void turn_left();
	void turn_right();
	void pose_correction();
private:
	ros::NodeHandle n;
	ros::Subscriber pose_correct;
	void poseMeassageReceived(const geometry_msgs::Twist &msg);
};