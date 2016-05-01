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
#include "kinect_slam/Pose2DMsg.h"


#define PI 3.14159265

// types for color clouds
//typedef pcl::PointXYZRGB Point;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
//typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> KinectSyncPolicy;


// types for non colour
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo> SimulationPolicy;
typedef message_filters::sync_policies::ApproximateTime<kinect_slam::Pose2DMsg, sensor_msgs::Image, sensor_msgs::CameraInfo> PioneerPolicy;

typedef boost::shared_ptr<kinect_slam::Pose2DMsg const> Pose2DMsgConstPtr;

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
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;

	message_filters::Subscriber<kinect_slam::Pose2DMsg> pioneer_sub; // pioneer state
	message_filters::Synchronizer<PioneerPolicy> pioneer_sync;

	//message_filters::Subscriber<nav_msgs::Odometry> simulation_sub;
	//age_filters::Synchronizer<SimulationPolicy> simulation_sync;

	//message_filters::Subscriber<sensor_msgs::Image> img_sub; // for color


public:
	PointCloudNode();
	~PointCloudNode(){};

	// real world call back, no color PoseStampedConstPtr
	void pioneer_callback(const kinect_slam::Pose2DMsgConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
	// real world, with color
	// void pioneer_callback


	// simulation callback, color
	//void simulation_callback(const nav_msgs::Odometry::ConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
	// simulation callback, no color
	void simulation_callback(const nav_msgs::Odometry::ConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);

	void cloud_append(PointCloudPtr);
	void build_octomap();

	// transformation helpers
	PointCloudPtr transform_cloud(PointCloudPtr, float, float, float, float, const std::string);
	PointCloudPtr to_global(PointCloudPtr);

	// filtering
	PointCloudPtr pt_filter(PointCloudPtr, const std::string, const float, const float);
	PointCloudPtr remove_floor(PointCloudPtr);
	void voxel_filter(float);

	// simulate data for testing methods
	PointCloudPtr simulate_circle(int, float);
	PointCloudPtr simulate_square(int, float);

	// methods to validate intermediate results
	void print_cloud(PointCloudPtr);
	void visualize_cloud(PointCloudPtr);
};
