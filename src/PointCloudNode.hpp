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
#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <cv_bridge/cv_bridge.h> // for use in converting sim data
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>


#define PI 3.14159265

// types for color clouds
//typedef pcl::PointXYZRGB Point;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;


// types for non colour
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Image> PioneerPolicy;

class PointCloudNode
{
private:
	int cloud_sz;
	int num_frames;
	PointCloudPtr cloud;
	Eigen::Vector3d state_mean;
	Eigen::Vector3d init_pose;

	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Image> dep_sub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> pioneer_sub;
	message_filters::Synchronizer<PioneerPolicy> pioneer_sync;
	//message_filters::Subscriber<sensor_msgs::Image> img_sub; // for color


public:
	PointCloudNode();
	~PointCloudNode(){};

	// real world call back, no color PoseStampedConstPtr
	void pioneer_callback(const geometry_msgs::PoseStampedConstPtr& state_msg, const sensor_msgs::ImageConstPtr&  dep);

	void cloud_append(PointCloudPtr new_cloud);
	void build_octomap();

	// transformation helpers
	PointCloudPtr transform_cloud(PointCloudPtr in_cloud, float dx, float dy, float dz, float theta, const std::string axis);
	PointCloudPtr to_global(PointCloudPtr in_cloud);

	// filtering
	PointCloudPtr pt_filter(PointCloudPtr in_cloud, const std::string field, const float min_range, const float max_range);
	PointCloudPtr remove_floor(PointCloudPtr in_cloud);
	void voxel_filter(float leafsize);

	// simulate data for testing methods
	PointCloudPtr simulate_circle(int, float);
	PointCloudPtr simulate_square(int, float);

	// methods to validate intermediate results
	void print_cloud(PointCloudPtr);
	void visualize_cloud(PointCloudPtr);
};
