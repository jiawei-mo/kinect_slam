#include "EKF_SLAM_Node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

EKF_SLAM_Node::EKF_SLAM_Node():
	vel_sub(nh, "/control", 1),
	lmk_sub(nh, "/landmarkWithDscrt", 1),
	sync(vel_sub, lmk_sub, 10)
{
	sync.registerCallback(boost::bind(&EKF_SLAM_Node::CtrlLmkCallback, this, _1, _2));
  slam_ptr = boost::shared_ptr<EKF_SLAM>(new EKF_SLAM());
	f = boost::bind(&EKF_SLAM_Node::updateConfig, this, _1, _2);
  server.setCallback(f);
}

void matToBitset(cv::Mat src, std::vector< boost::dynamic_bitset<> >& dst)
{
	for(int j=0; j<src.cols; j++)
	{
		boost::dynamic_bitset<> cur(src.rows);
		for(int i=0; i<src.rows; i++)
		{
			cur[i] = src.at<float>(i,j) > 0.5? true : false;
		}
		dst.push_back(cur);
	}
}

void EKF_SLAM_Node::CtrlLmkCallback(const kinect_slam::PioneerVelControlConstPtr& ctrl, const sensor_msgs::ImageConstPtr& lmk)
{
  double l_vel = ctrl->left_vel;
  double r_vel = ctrl->right_vel;

	cv_bridge::CvImagePtr img_ptr;
  try
  {
      img_ptr = cv_bridge::toCvCopy(lmk, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  } 
  cv::Mat kp_dsrct = img_ptr->image;
  cv::Mat measurements = kp_dsrct(cv::Rect(0, 0, kp_dsrct.cols, 3));
  cv::Mat _descriptors = kp_dsrct(cv::Rect(0, 3, kp_dsrct.cols, kp_dsrct.rows-3));
  std::vector< boost::dynamic_bitset<> > descriptors;
  matToBitset(_descriptors, descriptors);

  Eigen::MatrixXd measurementDB;
  std::vector< boost::dynamic_bitset<> > descriptorDB;
  slam_ptr->getMeasurement(measurementDB, descriptorDB);

  slam_ptr->predict(l_vel, r_vel);

  std::vector<cv::DMatch> matches;
  slam_ptr->landmark_match(measurements, measurementDB, descriptors, descriptorDB, matches, vertical_offset, max_horizontal_threshold, min_horizontal_threshold, dist_threshold);
    
}

void EKF_SLAM_Node::updateConfig(kinect_slam::KinectSLAMConfig &config, uint32_t level)
{
  min_horizontal_threshold = config.min_horizontal_threshold;
  max_horizontal_threshold = config.max_horizontal_threshold;
  vertical_offset = config.vertical_offset;
  dist_threshold = config.dist_threshold;
}