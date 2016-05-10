#include "map_server.hpp"


MapServerNode::MapServerNode(): 
  pcl_sub(nh, "/camera/rgb/image_color", 1),
  pose_sub(nh, "/camera/depth/image", 1),
	info_sub(nh, "/camera/depth/camera_info", 1),
	sync(KinectSyncPolicy(10), img_sub, dep_sub, info_sub)
{
  sync.registerCallback(boost::bind(&LandmarkExtractorNode::imageMessageCallback, this, _1, _2, _3));
  fd_ptr = boost::shared_ptr<HarrisDetector>(new HarrisDetector(7, 50, 50, 3, true, false, 7, 3));
  de_ptr = boost::shared_ptr<BRIEF>(new BRIEF(15, 3, 8));
  f = boost::bind(&LandmarkExtractorNode::updateConfig, this, _1, _2);
  server.setCallback(f);

  landmark_pub = nh.advertise<kinect_slam::LandmarkMsg>("landmarkWithDscrt", 50);
  raw_point_pub = nh.advertise<kinect_slam::LandmarkMsg>("raw_depth",50);

}

void MapServerNode::pclMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
{
}