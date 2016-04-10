#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

class Control
{
public:
	void go_straight();
	void turn_left();
	void turn_right();
};