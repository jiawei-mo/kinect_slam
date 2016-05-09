#include "ros/ros.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/PoseStamped.h"
#include <string.h>

class Truth_Node
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher t_pub;
public:
	void tfCB(const tf::tfMessageConstPtr& tfmsg)
	{
		int n_msg = tfmsg->transforms.size();
		for(int i=0; i<n_msg; i++)
		{
			char *msgID = (char*)tfmsg->transforms[i].child_frame_id.c_str();
			if(strcmp(msgID, "/kinect") == 0)
			{
				geometry_msgs::PoseStamped test_pose;
				test_pose.header.stamp = tfmsg->transforms[i].header.stamp;
				test_pose.header.frame_id = "/map";
				test_pose.pose.position.x = tfmsg->transforms[i].transform.translation.x;
				test_pose.pose.position.y = tfmsg->transforms[i].transform.translation.y;
				test_pose.pose.position.z = tfmsg->transforms[i].transform.translation.z;
				double x = tfmsg->transforms[i].transform.rotation.x;
				double y = tfmsg->transforms[i].transform.rotation.y;
				double z = tfmsg->transforms[i].transform.rotation.z;
				double w = tfmsg->transforms[i].transform.rotation.w;
				test_pose.pose.orientation.x = (w-z)/2;
				test_pose.pose.orientation.y = (x+y)/2;
				test_pose.pose.orientation.z = (y-x)/2;
				test_pose.pose.orientation.w = (z+w)/2;
			    t_pub.publish(test_pose);
			}
		}
	}
	Truth_Node()
	{
		sub = nh.subscribe("/tf", 10, &Truth_Node::tfCB, this);
		t_pub =nh.advertise<geometry_msgs::PoseStamped>("ture_pose", 50);
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker");
	Truth_Node tn;
	ros::spin();
	return 0;
}