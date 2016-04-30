#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Pose2D.h>  // for state message
#include <nav_msgs/Odometry.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <cv_bridge/cv_bridge.h> // for use in converting sim data
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>
#include <string>



#define PI 3.14159265

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> KinectSyncPolicy;

class PointCloudNode
{
private:
	int cloud_sz;
	int num_frames;
	PointCloudPtr cloud;
	Eigen::Vector3d state_mean;
	Eigen::Vector3d init_pose;

	ros::NodeHandle nh;
	ros::Subscriber state_sub;

	message_filters::Subscriber<sensor_msgs::Image> img_sub;
	message_filters::Subscriber<sensor_msgs::Image> dep_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
	message_filters::Synchronizer<KinectSyncPolicy> sync;

public:
	PointCloudNode();
	~PointCloudNode(){};

	void pcl_callback(const sensor_msgs::PointCloud2ConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
	void sim_pcl_callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);

	void state_callback(const geometry_msgs::Pose2D&);
	void sim_state_callback(const nav_msgs::Odometry::ConstPtr&);

	void cloud_append(PointCloudPtr);
	void print_cloud(PointCloudPtr);
	void visualize_cloud(PointCloudPtr);

	// simulate data for testing methods
	PointCloudPtr simulate_circle(int, float);
	PointCloudPtr simulate_square(int, float);

	// filtering
	PointCloudPtr pt_filter(PointCloudPtr, const std::string, const float, const float);
	void voxel_filter(PointCloudPtr, float);

	// transformation
	PointCloudPtr transform_cloud(PointCloudPtr, float, float, float, float, const std::string);
	PointCloudPtr to_global(PointCloudPtr);

	// octomap builder
	void build_octomap();
};
