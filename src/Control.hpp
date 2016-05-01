#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
class Control
{
public:
	void pose_correction(double theta);
private:
	ros::NodeHandle n;
};