#include "ros/ros.h"
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>

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

	std::vector< std::vector<double> > map;

public:
	MapServerNode();
	~MapServerNode(){};
	void pclMessageCallback(const PointCloud::ConstPtr& pcl_pts, const geometry_msgs::PoseStampedConstPtr& pose);
};