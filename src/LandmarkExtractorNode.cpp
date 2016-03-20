#include "ros/ros.h"
#include "LandmarkExtractorNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>
LandmarkExtractorNode::LandmarkExtractorNode(): 
  it(nh),
  img_sub(nh, "/kinect2/qhd/image_color_rect", 1),
  dep_sub(nh, "/kinect2/qhd/image_depth_rect", 1),
	info_sub(nh, "/kinect2/qhd/camera_info", 1),
	sync(img_sub, dep_sub, info_sub, 10)
{
  sync.registerCallback(boost::bind(&LandmarkExtractorNode::imageMessageCallback, this, _1, _2, _3));
  fd_ptr = boost::shared_ptr<HarrisDetector>(new HarrisDetector(7, 50, 50, 3, true, false, 7, 3));
  de_ptr = boost::shared_ptr<BRIEF>(new BRIEF(15, 3, 8));
  f = boost::bind(&LandmarkExtractorNode::updateConfig, this, _1, _2);
  server.setCallback(f);
  landmark_pub = it.advertise("landmarkWithDscrt", 1);
}

void betset_to_matrix(std::vector< boost::dynamic_bitset<> > dscrt, cv::Mat& res)
{
	int rn = dscrt[0].size();
	int cn = dscrt.size();
	cv::Mat dscrtMat(rn, cn, CV_32F);
	for(int j=0; j<cn; j++)
	{
		for(int i=0; i<rn; i++)
		{
			dscrtMat.at<float>(i,j) = dscrt[j][i]? 1 : 0;
		}
	}	
	res = dscrtMat;
}

void LandmarkExtractorNode::imageMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
{
    cv_bridge::CvImagePtr img_ptr, dep_ptr;
    cv::Mat gry_img;
    try
    {
        img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        dep_ptr = cv_bridge::toCvCopy(dep, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } 

    cv::Mat clr_img = img_ptr->image;
    cv::Mat depth = dep_ptr->image;

    cvtColor(clr_img, gry_img, CV_BGR2GRAY);

    std::vector<cv::KeyPoint> kp;
    fd_ptr->detect(gry_img, kp);

    for(int j=0;j<kp.size();j++)
    {
      circle(clr_img, kp[j].pt, 5, CV_RGB(255,0,0));
    }

    std::vector< boost::dynamic_bitset<> > dscrt;
    de_ptr->extract(gry_img, kp, dscrt);
    cv::Mat dscrtMat;
    betset_to_matrix(dscrt, dscrtMat);

    double fx = info->K[0];
    double cx = info->K[2];
    double fy = info->K[4];
    double cy = info->K[5];

    cv::Mat kp_measurement(3+dscrtMat.rows, kp.size(), CV_32F);
    for(int i=0; i<kp.size(); i++)
    {
    	double z = depth.at<float>(kp[i].pt.y, kp[i].pt.x);
    	double x = z * (kp[i].pt.x - cx) / fx;
    	double y = z * (kp[i].pt.y - cy) / fy;
      kp_measurement.at<float>(0, i) = sqrt(x*x + z*z);
      kp_measurement.at<float>(1, i) = atan2(z, x);
      kp_measurement.at<float>(2, i) = - y;
    }
   	dscrtMat.copyTo(kp_measurement(cv::Rect(0,3,dscrtMat.cols, dscrtMat.rows)));
    sensor_msgs::ImagePtr landmark_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", kp_measurement).toImageMsg();
    landmark_pub.publish(landmark_msg);
}

void LandmarkExtractorNode::updateConfig(kinect_slam::KinectSLAMConfig &config, uint32_t level)
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