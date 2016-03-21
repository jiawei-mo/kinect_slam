#include "EKF_SLAM_Node.hpp"
#include <iostream>
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

void EKF_SLAM_Node::CtrlLmkCallback(const kinect_slam::PioneerVelControlConstPtr& ctrl, const kinect_slam::LandmarkMsgConstPtr& lmk)
{
  double l_vel = ctrl->left_vel;
  double r_vel = ctrl->right_vel;

  int landmark_count = lmk->landmark_count;
  int descriptor_len = lmk->descriptor_len;
  Eigen::MatrixXd measurements(3,landmark_count);
  std::vector< boost::dynamic_bitset<> > descriptors;
  int d_c = 0;
  for(int j=0; j<landmark_count; j++)
  {
    measurements(0,j) = lmk->position_x[j];
    measurements(1,j) = lmk->position_y[j];
    measurements(2,j) = lmk->position_signature[j];
    boost::dynamic_bitset<> cur(descriptor_len);
    for(int i=0; i<descriptor_len; i++)
    {
      cur[i] = lmk->descriptor_mat[d_c++]>0? true : false;
    }
    descriptors.push_back(cur);
  }

  slam_ptr->predict(l_vel, r_vel);

  std::vector<std::array<size_t, 3> > matches;
  slam_ptr->landmark_match(measurements, descriptors, matches, max_signature_threshold, match_threshold);
  std::cout<<matches.size()<<std::endl;
  bool flags[landmark_count];
  for(int i=0; i<landmark_count; i++) flags[i] = false;
  for(int i=0; i<matches.size(); i++)
  {
    flags[i] = true;
    if(matches[i][2] > new_landmark_threshold)
      slam_ptr->add_landmark(measurements(0, matches[i][0]), measurements(1, matches[i][0]), measurements(2, matches[i][0]), descriptors[matches[i][0]]);
    else
    {
      Eigen::Vector3d matched_measurement;
      matched_measurement << measurements(0, matches[i][0]), measurements(1, matches[i][0]), measurements(2, matches[i][0]);
      slam_ptr->measurement_update(matched_measurement, matches[i][1]);
    }
  }
  for(int i=0; i<landmark_count; i++)
  {
    if(!flags[i])
    {
      slam_ptr->add_landmark(measurements(0,i), measurements(1,i), measurements(2,i), descriptors[i]);
    }
  }
}

void EKF_SLAM_Node::updateConfig(kinect_slam::KinectSLAMConfig &config, uint32_t level)
{
  max_signature_threshold = config.max_signature_threshold;
  match_threshold = config.match_threshold;
  new_landmark_threshold = config.new_landmark_threshold;
}