#include "ros/ros.h"
#include "LandmarkExtractorNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#define MIN_DEPTH_MM 10

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

LandmarkExtractorNode::LandmarkExtractorNode(): 
  img_sub(nh, "/camera/rgb/image_rect_color", 1),
  dep_sub(nh, "/camera/depth/image_rect", 1),
	info_sub(nh, "/camera/rgb/camera_info", 1),
	sync(KinectSyncPolicy(10), img_sub, dep_sub, info_sub)
{
  sync.registerCallback(boost::bind(&LandmarkExtractorNode::imageMessageCallback, this, _1, _2, _3));
  fd_ptr = boost::shared_ptr<HarrisDetector>(new HarrisDetector(7, 50, 50, 3, true, false, 7, 3));
  de_ptr = boost::shared_ptr<BRIEF>(new BRIEF(15, 3, 8));
  f = boost::bind(&LandmarkExtractorNode::updateConfig, this, _1, _2);
  server.setCallback(f);

  pcl_pub = nh.advertise<PointCloud> ("points2", 1);
  landmark_pub = nh.advertise<kinect_slam::LandmarkMsg>("landmarkWithDscrt", 50);
}

void LandmarkExtractorNode::imageMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
{
  cv_bridge::CvImageConstPtr img_ptr, dep_ptr;
  cv::Mat gry_img;
  try
  {
      img_ptr = cv_bridge::toCvShare(img, img->encoding);
      dep_ptr = cv_bridge::toCvShare(dep, dep->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  } 

  cv::Mat clr_img, depth;
  img_ptr->image.copyTo(clr_img);
  dep_ptr->image.copyTo(depth);
  //ERROR: depth seems incorrect(to small: e^-30)
  // std::cout<<depth<<std::endl;

  cvtColor(clr_img, gry_img, CV_BGR2GRAY);

  std::vector<cv::KeyPoint> kp;
  fd_ptr->detect(gry_img, kp);
  if(kp.size()==0) return; 
  
  for(int j=0;j<kp.size();j++)
  {
    circle(clr_img, kp[j].pt, 5, CV_RGB(255,0,0));
  }

  cv::namedWindow("Feature Points");
  cv::imshow("Feature Points", clr_img);
  cv::waitKey(3);

  std::vector< boost::dynamic_bitset<> > dscrt;
  de_ptr->extract(gry_img, kp, dscrt);

  kinect_slam::LandmarkMsg new_measurement_msg;

  double fx = info->K[0];
  double cx = info->K[2];
  double fy = info->K[4];
  double cy = info->K[5];
  new_measurement_msg.position_x.clear();
  new_measurement_msg.position_y.clear();
  new_measurement_msg.position_signature.clear();
  new_measurement_msg.landmark_count = 0;
  new_measurement_msg.descriptor_len = dscrt[0].size();

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "map";
  msg->height = 1;
  msg->width = kp.size();
  msg->points.resize(msg->height * msg->width);
  msg->is_dense = false;
  for(int i=0; i<kp.size(); i++)
  {
  	unsigned short zt = depth.at<unsigned short>(kp[i].pt.y, kp[i].pt.x);
    if(zt<MIN_DEPTH_MM ) continue;
    double z = static_cast<double>(zt);
  	double x = z * (kp[i].pt.x - cx) / fx;
  	double y = z * (kp[i].pt.y - cy) / fy;
    new_measurement_msg.position_x.push_back(z);
    new_measurement_msg.position_y.push_back(-x);
    new_measurement_msg.position_signature.push_back(-y);
    for(int j=0; j<dscrt[i].size(); j++) new_measurement_msg.descriptor_mat.push_back(dscrt[i][j]? 1.0 : 0.0);
    new_measurement_msg.landmark_count++;

    msg->points[i].x = x/1000.0;
    msg->points[i].y = y/1000.0;
    msg->points[i].z = z/1000.0;
  }
 	landmark_pub.publish(new_measurement_msg);
  // std::cout<<msg->points.size()<<std::endl;
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pcl_pub.publish(msg);
  // std::cout<<new_measurement_msg.descriptor_mat.size()<<std::endl;
}

void LandmarkExtractorNode::updateConfig(kinect_slam::LandmarkExtractorConfig &config, uint32_t level)
{
    int hws = config.harris_window_size; 
    bool haf = config.harris_anms_flag;
    int har = config.harris_anms_radius;
    bool hff = config.harris_fix_number_flag;
    int hnp = config.harris_number_of_points;
    float hrt = config.harris_response_threshold;
    int hbs = config.harris_blur_size;
    int hbv = config.harris_blur_variance;

    int dps = config.descriptor_patch_size;
    int dbs = config.descriptor_brief_size;
    int dss = config.descriptor_smooth_size;

    fd_ptr = boost::shared_ptr<HarrisDetector>(new HarrisDetector(hws, hrt, hnp, har, hff, haf, hbs, hbv));
    de_ptr = boost::shared_ptr<BRIEF>(new BRIEF(dps, dss, dbs));
}