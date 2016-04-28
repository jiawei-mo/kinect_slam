#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
class Control
{
public:
	void pose_correction();
private:
	ros::NodeHandle n;
	ros::Subscriber pose_correct;
	void poseMeassageReceived(const geometry_msgs::Pose2D &msg);
};