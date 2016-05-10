#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, geometry_msgs::PoseStamped> MapSyncPolicy;

class MapServerNode
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<PointCloud> pcl_sub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
	message_filters::Synchronizer<MapSyncPolicy> sync;
	ros::Publisher map_pub;

public:
	MapServerNode();
	~MapServerNode(){};
	void pclMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info);
}