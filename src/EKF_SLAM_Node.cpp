#include "EKF_SLAM_Node.hpp"
#include <iostream>
EKF_SLAM_Node::EKF_SLAM_Node()
{
  ini_flag=1;
 // get_ini_time = nh.subscribe("/ini_time",1, &EKF_SLAM_Node::Initialize_Time_Recieved,this);
  repub_sub = nh.subscribe("/control", 1, &EKF_SLAM_Node::Ctrl_republish,this);
  vel_sub = nh.subscribe("/control_repub", 1, &EKF_SLAM_Node::CtrlCallback,this);
  lmk_sub = nh.subscribe("/landmarkWithDscrt", 1, &EKF_SLAM_Node::LmkCallback,this);
  slam_ptr = boost::shared_ptr<EKF_SLAM>(new EKF_SLAM());
	f = boost::bind(&EKF_SLAM_Node::updateConfig, this, _1, _2);
  server.setCallback(f);
}

//for propagation
void EKF_SLAM_Node::CtrlCallback(const geometry_msgs::TwistStamped& ctrl)
{
  if(ini_flag==1)
  {
    pre_time_stamp=ros::Time::now();
    ini_flag=0;
  }
  double l_vel = ctrl.twist.linear.x;
  double r_vel = ctrl.twist.angular.z;
  ros::Time current_time_stamp = ctrl.header.stamp;
  //double delta_t = (current_time_stamp.sec - pre_time_stamp.sec)+(current_time_stamp.nsec - pre_time_stamp.nsec)*1e-9;
  double delta_t = (current_time_stamp.sec - pre_time_stamp.sec);
  // if (current_time_stamp>0)
  // {
     ROS_INFO_STREAM("Control data receieved");
     std::cout<<"Delta_T is  "<<delta_t<<"s\n";
  // }
  slam_ptr->predict(l_vel, r_vel, delta_t);
  pre_time_stamp = current_time_stamp;
}

void EKF_SLAM_Node::Ctrl_republish(const geometry_msgs::TwistStamped& ctrl)
{
  ros::Publisher repub=nh.advertise<geometry_msgs::TwistStamped>("/control_repub",1);
  geometry_msgs::TwistStamped msg_pub;
  double CLOCK_SPEED = 20;
  ros::Rate rate(CLOCK_SPEED);
  msg_pub.twist=ctrl.twist;
  while(ros::ok())
  {
    msg_pub.header.stamp=ros::Time::now();
    repub.publish(msg_pub);
    ros::spinOnce();
    rate.sleep();
  }
}

//for update
void EKF_SLAM_Node::LmkCallback(const kinect_slam::LandmarkMsgConstPtr& lmk)
{
	/* create BRIEF descriptor */

  //std::cout<<"start: "<<std::endl;
  int landmark_count = lmk->landmark_count;
  int descriptor_len = lmk->descriptor_len;
  Eigen::MatrixXd measurements(3,landmark_count);
  std::vector< boost::dynamic_bitset<> > descriptors;
  int d_c = 0;
  for(int j=0; j<landmark_count; j++)
  {
    // std::cout<<"x: "<<lmk->position_x[j]<<" y: "<<lmk->position_y[j]<<" z: "<<lmk->position_signature[j]<<std::endl;
    measurements(0,j) = lmk->position_x[j]; /* x is distance from to object perpendicular to image plane parallel to ground */
    // std::cout<<"x: "<<lmk->position_x[j];
    measurements(1,j) = lmk->position_y[j]; /* y distance from center along image plane parallel to ground */
    // std::cout<<" y: "<<lmk->position_y[j];
    measurements(2,j) = lmk->position_signature[j]; /* z height distance from image center to point perpendicular to ground */
    // std::cout<<" signature: "<<lmk->position_signature[j]<<std::endl;
    boost::dynamic_bitset<> cur(descriptor_len);
    for(int i=0; i<descriptor_len; i++)
    {
      cur[i] = lmk->descriptor_mat[d_c++]>0? true : false;
    }
    descriptors.push_back(cur);
  // std::cout<<"x: "<<measurements(0,j)<<" y: "<<measurements(1,j)<<" z: "<<measurements(2,j)<<std::endl;
  }

  /* find matches */
  std::vector<std::array<size_t, 3> > matches, new_points; /* Vector of array with elmts [lmk_index_org, lmk_index_hist, sim_meas] */
  slam_ptr->landmark_match(measurements, descriptors, matches, new_points, max_signature_threshold, match_threshold, new_points_threshold); /* gets the matches */
  //std::cout<<matches.size()<<std::endl;
  
  /* The flag determines which landmark matches are new. flag = true => correspondance; flag = false => new */
  std::cout<<"Total points: "<<landmark_count<<" Matched points: "<<matches.size()<<" New points: "<<new_points.size()<<std::endl;
  
  /* add matched elements to the H stack */
  Eigen::VectorXd matched_measurement(3*matches.size());
  Eigen::VectorXd matched_idx(matches.size());
  for(int i=0; i<matches.size(); i++)
  {
    // std::cout<<"dist:"<<matches[i][2]<<std::endl;
    matched_measurement(3*i) = measurements(0, matches[i][0]);
    matched_measurement(3*i+1) = measurements(1, matches[i][0]);
    matched_measurement(3*i+2) = measurements(2, matches[i][0]);
    matched_idx(i) = matches[i][1];
  }

   slam_ptr->measurement_update(matched_measurement, matched_idx); /* add the landmark to the H stack */


  /* New landmarks are added to the history of landmarks */
  for(int i=0; i<new_points.size(); i++)
  {
      slam_ptr->add_landmark(measurements(0,new_points[i][0]), measurements(1,new_points[i][0]), measurements(2,new_points[i][0]), descriptors[new_points[i][0]]);
  }
  slam_ptr->landmark_pcl_pub();
  slam_ptr->print_state();
}

void EKF_SLAM_Node::updateConfig(kinect_slam::EKFSLAMConfig &config, uint32_t level)
{
  max_signature_threshold = config.max_signature_threshold;
  match_threshold = config.match_threshold;
  new_points_threshold = config.new_points_threshold;
}