#include <dynamic_reconfigure/server.h>
#include "StereoMatcher.hpp"
#include "HarrisDetector.hpp"
#include "BRIEF.hpp"
#include "kinect_slam/KinectSLAMConfig.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class LandmarkMatcherNode
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> img_sub;
	message_filters::Subscriber<sensor_msgs::Image> dep_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync;

	boost::shared_ptr<HarrisDetector> fd_ptr;
	boost::shared_ptr<BRIEF> de_ptr;
	boost::shared_ptr<StereoMatcher> sm_ptr;
	dynamic_reconfigure::Server<kinect_slam::KinectSLAMConfig> server;
	dynamic_reconfigure::Server<kinect_slam::KinectSLAMConfig>::CallbackType f;

	// std::vector< boost::dynamic_bitset<> > last_dscrt;
 // 	std::vector<cv::KeyPoint> last_kp;
 // 	cv::Mat last_kp_xyz;

	int vertical_offset;
	int max_horizontal_threshold;
  int min_horizontal_threshold;

	tf2_ros::TransformBroadcaster br;

public:
	LandmarkMatcherNode();
	~LandmarkMatcherNode(){};
	void imageMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info);
	void updateConfig(kinect_slam::KinectSLAMConfig &config, uint32_t level);
};