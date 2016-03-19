#include "ros/ros.h"
#include "LandmarkMatcherNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>
LandmarkMatcherNode::LandmarkMatcherNode(): 
    img_sub(nh, "/kinect2/qhd/image_color_rect", 1),
    dep_sub(nh, "/kinect2/qhd/image_depth_rect", 1),
	info_sub(nh, "/kinect2/qhd/camera_info", 1),
	sync(img_sub, dep_sub, info_sub, 10)
{
    sync.registerCallback(boost::bind(&LandmarkMatcherNode::imageMessageCallback, this, _1, _2, _3));
    fd_ptr = boost::shared_ptr<HarrisDetector>(new HarrisDetector(7, 50, 50, 3, true, false, 7, 3));
    de_ptr = boost::shared_ptr<BRIEF>(new BRIEF(15, 3, 8));
    sm_ptr = boost::shared_ptr<StereoMatcher>(new StereoMatcher(0.75, 500.0));
    f = boost::bind(&LandmarkMatcherNode::updateConfig, this, _1, _2);
    server.setCallback(f);

    br = tf2_ros::TransformBroadcaster();

}

void LandmarkMatcherNode::imageMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
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

    cv::Mat kp_xyz(3, kp.size(), CV_32F);
    for(int i=0; i<kp.size(); i++)
    {
        kp_xyz.at<float>(0, i) = kp[i].pt.x;
        kp_xyz.at<float>(1, i) = depth.at<float>(kp[i].pt.y, kp[i].pt.x);
        kp_xyz.at<float>(2, i) = - kp[i].pt.y;
    }

    std::vector< boost::dynamic_bitset<> > dscrt;
    de_ptr->extract(gry_img, kp, dscrt);

    // std::vector<cv::DMatch> matches;
    // sm_ptr->disparity_match(kp, last_kp, dscrt, last_dscrt, matches, vertical_offset, max_horizontal_threshold, min_horizontal_threshold);
    // sm_ptr->drawDisparity(clr_img, kp, last_kp, matches);

    // last_kp = kp;
    // last_dscrt = dscrt;
    // last_kp_xyz = kp_xyz;
}

void LandmarkMatcherNode::updateConfig(kinect_slam::KinectSLAMConfig &config, uint32_t level)
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

    int min_t_th = config.min_horizontal_threshold;
    int max_t_th = config.max_horizontal_threshold;

    int tvo = config.vertical_offset;

    double mt = config.matcher_dist_threshold;
    double tf_th = config.tf_threshold;

    vertical_offset = tvo;
    max_horizontal_threshold = max_t_th;
    min_horizontal_threshold = min_t_th;

    fd_ptr = boost::shared_ptr<HarrisDetector>(new HarrisDetector(hws, hrt, hnp, har, hff, haf, hbs, hbv));
    de_ptr = boost::shared_ptr<BRIEF>(new BRIEF(dps, dss, dbs));
    sm_ptr = boost::shared_ptr<StereoMatcher>(new StereoMatcher(mt, tf_th));
}