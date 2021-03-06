#include "ros/ros.h"
#include "LandmarkExtractorNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>

LandmarkExtractorNode::LandmarkExtractorNode():
  img_sub(nh, "/camera/rgb/image_rect_color", 1),
  dep_sub(nh, "/camera/depth/image_rect", 1),
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
  pcl_current_frame_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("pcl_current_frame", 50);
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

  std::vector< boost::dynamic_bitset<> > dscrt;
  de_ptr->extract(gry_img, kp, dscrt);

  kinect_slam::LandmarkMsg new_measurement_msg;
  kinect_slam::LandmarkMsg raw_measurement_msg;

  double fx = info->K[0];
  double cx = info->K[2];
  double fy = info->K[4];
  double cy = info->K[5];
  new_measurement_msg.position_x.clear();
  new_measurement_msg.position_y.clear();
  new_measurement_msg.position_signature.clear();
  new_measurement_msg.landmark_count = 0;
  new_measurement_msg.descriptor_len = dscrt[0].size();

  //publish all points
  for(int i=0;i<depth.rows/2;i++)
  {
    for(int j=0;j<depth.cols/3;j++)
    {
      double z = depth.at<float>(i,j); //only collect point for left wall
      if(z<2.5)
      {
        double x = z * (j - cx) / fx;
        double y = z * (i - cy) / fy;
        raw_measurement_msg.position_x.push_back(z);
        raw_measurement_msg.position_y.push_back(-x);
        raw_measurement_msg.position_signature.push_back(-y);
        raw_measurement_msg.landmark_count++;
      }
    }
  }
  if(raw_measurement_msg.landmark_count<=100)
  {
     raw_measurement_msg.position_x.clear();
   raw_measurement_msg.position_y.clear();
   raw_measurement_msg.position_signature.clear();
   raw_measurement_msg.landmark_count=0;
   for(int i=depth.rows/2;i<depth.rows;i++)
   {
    for(int j=depth.cols*2/3;j<depth.cols;j++)
    {
      double z = depth.at<float>(i,j); //only collect point for left wall
      if(z<2.5)
      {
        double x = z * (j - cx) / fx;
        double y = z * (i - cy) / fy;
        raw_measurement_msg.position_x.push_back(z);
        raw_measurement_msg.position_y.push_back(-x);
        raw_measurement_msg.position_signature.push_back(-y);
        raw_measurement_msg.landmark_count++;
      }
    }
  }
  }
  //
  // std::cout <<raw_measurement_msg.landmark_count<<std::endl;
  raw_point_pub.publish(raw_measurement_msg);
  //
  for(int i=0; i<kp.size(); i++)
  {
    double z = depth.at<float>(kp[i].pt.y, kp[i].pt.x);
    if(z>MIN_DEPTH && z<MAX_DEPTH)
    {
      // std::cout<<"Depth: "<<z<<std::endl;
      double x = z * (kp[i].pt.x - cx) / fx;
      double y = z * (kp[i].pt.y - cy) / fy;
      new_measurement_msg.position_x.push_back(z);
      new_measurement_msg.position_y.push_back(-x);
      new_measurement_msg.position_signature.push_back(-y);
      for(int j=0; j<dscrt[i].size(); j++) new_measurement_msg.descriptor_mat.push_back(dscrt[i][j]? 1.0 : 0.0);
      new_measurement_msg.landmark_count++;
      circle(clr_img, kp[i].pt, 5, CV_RGB(255,0,0));
    }
  }

  cv::namedWindow("Feature Points");
  cv::imshow("Feature Points", clr_img);
  cv::waitKey(3);
  landmark_pub.publish(new_measurement_msg);



  // BUILD POINT CLOUD
  cv::patchNaNs(depth, 0.0);
  PointCloudPtr pointcloud_msg (new PointCloud);
  Point pt;
  for (int i = 0; i < depth.rows; i+=16) {
    for (int j = 0; j < depth.cols; j+=16) {
      double z = depth.at<float>(i,j);
      if (z > MIN_DEPTH && z < MAX_DEPTH) {
        double x = z * (j - cx) / fx;
        double y = z * (i - cy) / fy;
        pt.x = z;
        pt.y = -x;
        pt.z = -y;
        if(pt.z < -0.1) continue; 
        pointcloud_msg->points.push_back(pt);
      }
    }
  }
  pointcloud_msg->height = 1;
  pointcloud_msg->width = pointcloud_msg->points.size();
  // publish the point cloud
  pointcloud_msg->header.frame_id = "/map";
  pcl_conversions::toPCL(ros::Time::now(), pointcloud_msg->header.stamp);

  // apply filtering to downsample point cloud further

  /*
  // Divide space into voxels, replaces points within a voxels by their centroids.
  pcl::VoxelGrid<Point> sor;
  sor.setInputCloud(pointcloud_msg);
  // one cell within each "leafsize" meters
  float leafsize = 0.1;
  sor.setLeafSize(leafsize, leafsize, leafsize);
  sor.filter(*pointcloud_msg);
  */

  // remove floor
  // float min_z = 99999.0;
  // for(size_t i=0; i < pointcloud_msg->points.size(); ++i) {
  //   if (pointcloud_msg->points[i].z < min_z ) {
  //     min_z = pointcloud_msg->points[i].z;
  //   }
  // }
  // PointCloudPtr no_floor_cloud (new PointCloud());
  // pcl::PassThrough<Point> floor_filter;
  // floor_filter.setInputCloud(pointcloud_msg);
  // floor_filter.setFilterFieldName("z");
  // floor_filter.setFilterLimits(min_z + .1, min_z + 2.0);
  // floor_filter.filter(*no_floor_cloud);

  // publish filtered cloud
  pcl_current_frame_pub.publish(pointcloud_msg);
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

    MIN_DEPTH = config.kinect_min_depth;
    MAX_DEPTH = config.kinect_max_depth;

    fd_ptr = boost::shared_ptr<HarrisDetector>(new HarrisDetector(hws, hrt, hnp, har, hff, haf, hbs, hbv));
    de_ptr = boost::shared_ptr<BRIEF>(new BRIEF(dps, dss, dbs));
}
