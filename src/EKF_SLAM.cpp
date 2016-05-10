#include "EKF_SLAM.hpp"
#include <iostream>
EKF_SLAM::EKF_SLAM()
{
	state_mean = Eigen::Vector3d::Zero();
	state_cov = Eigen::Matrix3d::Zero();
	G_accu = Eigen::Matrix3d::Identity(3,3);
	accu_flag = false;
	num_landmarks = 0;
	delta_t=0.25;

	R << KINECT_X_VAR*KINECT_X_VAR, 0, 0,
	 0, KINECT_Y_VAR*KINECT_Y_VAR, 0,
	 0, 0, KINECT_S_VAR*KINECT_S_VAR;

	 pcl_pub = nh.advertise<PointCloud> ("points2", 1);
	 robot_state_pub = nh.advertise<geometry_msgs::PoseStamped>("pose1", 50);
}

EKF_SLAM::EKF_SLAM(Eigen::Vector3d _mean, Eigen::Matrix3d _cov)
{
	state_mean = _mean;
	state_cov = _cov;
	G_accu = Eigen::Matrix3d::Identity(3,3);
	accu_flag = false;
	num_landmarks = 0;

	R << KINECT_X_VAR*KINECT_X_VAR, 0, 0,
	 0, KINECT_Y_VAR*KINECT_Y_VAR, 0,
	 0, 0, KINECT_S_VAR*KINECT_S_VAR;

	pcl_pub = nh.advertise<PointCloud> ("points2", 1);
	robot_state_pub = nh.advertise<geometry_msgs::PoseStamped>("pose1", 50);

}

//TODO
void EKF_SLAM::predict(double linear_vel, double angular_vel, double delta_t)
{
	// std::cout<<"Prediction: Vel: "<<linear_vel<<" Rot: "<<angular_vel<<std::endl;
	// std::cout<<"Before: "<<std::endl<<state_mean.block<3,1>(0,0)<<std::endl;
	double delta_x;
	double delta_y;
	double delta_theta;
	Eigen::Vector3d delta_state;
	Eigen::Matrix3d G;
	Eigen::MatrixXd V(3,2);

	delta_x = linear_vel*cos(state_mean(2))*delta_t;
	delta_y = linear_vel*sin(state_mean(2))*delta_t;
	delta_theta = angular_vel*delta_t;

    G<< 1, 0, (-linear_vel*delta_t*sin(state_mean(2))),
        0, 1, (linear_vel*delta_t*cos(state_mean(2))),
        0, 0, 1;

    V<< -cos(state_mean(2))*delta_t, 0,
        -sin(state_mean(2))*delta_t, 0,
        0, -delta_t;

	delta_state << delta_x,
				   delta_y,
				   delta_theta;
	state_mean.block<3,1>(0,0) += delta_state;
	if(state_mean(2)>=2*PI)
	{
		state_mean(2)=state_mean(2)-2*PI;
	}
	else if(state_mean(2)<0)
	{
		state_mean(2)=2*PI+state_mean(2);
	}
	else
	{
    }
    double sigma_l =  MOTION_FACTOR*linear_vel;
    double sigma_r = TURN_FACTOR*angular_vel;

	Eigen::Matrix2d cov_control;
	cov_control << sigma_l*sigma_l, 0,
				   0, sigma_r*sigma_r;

	state_cov.block<3,3>(0,0) = G * state_cov.block<3,3>(0,0) * G.transpose() + V * cov_control * V.transpose();
	state_cov.block<3,3>(0,0) = (state_cov.block<3,3>(0,0) + state_cov.block<3,3>(0,0).transpose()) / 2;
	G_accu = G * G_accu;
	accu_flag = true;
	// std::cout<<"After: "<<std::endl<<state_mean.block<3,1>(0,0)<<std::endl;

	//test_prediction
	geometry_msgs::PoseStamped test_pose;
	ros::Time new_now = ros::Time::now();
	test_pose.header.stamp = ros::Time(new_now.sec, new_now.nsec);
	test_pose.header.frame_id = "/map";
	test_pose.pose.position.x = state_mean(0);
	test_pose.pose.position.y = state_mean(1);
	test_pose.pose.position.z = 0;
	test_pose.pose.orientation.z = sin(state_mean(2)/2);
	test_pose.pose.orientation.w = cos(state_mean(2)/2);
    robot_state_pub.publish(test_pose);

    // write covariance and state estimate to file every couple of iterations
	std::vector<double> current_state = state_to_vector();
	propagation_history.push_back(current_state);
	if (propagation_history.size() % 10 == 0) {
		// default location of file: ~/.ros/filename.csv
		write_to_csv("propagation.csv", propagation_history);
	}

}

void EKF_SLAM::add_landmark(double x, double y, double sig, boost::dynamic_bitset<> dscrt)
{
	int former_length = state_mean.rows();

	state_mean.conservativeResize(former_length+3);
	state_mean(former_length) = state_mean(0) + (x+KINECT_DISP)*cos(state_mean(2)) - y*sin(state_mean(2));
	state_mean(former_length+1) = state_mean(1) + (x+KINECT_DISP)*sin(state_mean(2)) + y*cos(state_mean(2));
	state_mean(former_length+2) = sig;

	state_cov.conservativeResize(former_length+3, former_length+3);
	Eigen::MatrixXd H_Li, H_R;
	H_R = Eigen::MatrixXd::Zero(3,3);
	H_Li = Eigen::MatrixXd::Zero(3,3);
	H_R(0,0) = -cos(state_mean(2));
	H_R(0,1) = -sin(state_mean(2));
	H_R(1,0) =  sin(state_mean(2));
	H_R(1,1) = -cos(state_mean(2));
	H_R(0,2) =  y;
	H_R(1,2) = -x;
	H_Li(0,0) =  cos(state_mean(2));
	H_Li(0,1) =  sin(state_mean(2));
	H_Li(1,0) = -sin(state_mean(2));
	H_Li(1,1) =  cos(state_mean(2));
	H_Li(2,2) = 1;
	Eigen::MatrixXd _HHP = -H_Li.transpose()*H_R*state_cov.block(0,0,3,former_length);
	Eigen::MatrixXd HHPH_RH = H_Li.transpose()*(H_R*state_cov.block<3,3>(0,0)*H_R.transpose() + R)*H_Li;
	state_cov.block(former_length,0,3,former_length) = _HHP;
	state_cov.block(0,former_length,former_length,3) = _HHP.transpose();
	state_cov.block<3,3>(former_length,former_length) = HHPH_RH;

	descriptorDB.push_back(dscrt);
	num_landmarks++;
}

/* EKF SLAM update */
void EKF_SLAM::measurement_update(Eigen::VectorXd measurements, Eigen::VectorXd measurements_idx)
{
	// std::cout<<"Update"<<std::endl;
	if(accu_flag)
	{
		//multiply G_accu into covariance
		Eigen::MatrixXd Cov_update;
		Cov_update = G_accu * state_cov.block(0,3,3,state_mean.rows()-3);
		state_cov.block(0,3,3,state_mean.rows()-3) = Cov_update;
		state_cov.block(3,0,state_mean.rows()-3,3) = Cov_update.transpose();
		G_accu = Eigen::Matrix3d::Zero();
		accu_flag = false;
	}

	//std::cout<<"start"<<std::endl;
	int mm_count = measurements_idx.rows();
	Eigen::MatrixXd H_accu(3*mm_count, state_mean.rows());
	Eigen::MatrixXd R_accu(3*mm_count, 3*mm_count);
	Eigen::VectorXd _measurements(3*mm_count);
	for(int mm_i=0; mm_i<mm_count; mm_i++)
	{
		int landmark_idx = measurements_idx(mm_i);
		double q_x =  (state_mean(3+landmark_idx*3) - state_mean(0))*cos(state_mean(2)) + (state_mean(3+landmark_idx*3+1) - state_mean(1))*sin(state_mean(2)) - KINECT_DISP;
		double q_y = -(state_mean(3+landmark_idx*3) - state_mean(0))*sin(state_mean(2)) + (state_mean(3+landmark_idx*3+1) - state_mean(1))*cos(state_mean(2));
		_measurements(3*mm_i)=q_x;
		_measurements(3*mm_i+1)=q_y;
		_measurements(3*mm_i+2)=state_mean(3+landmark_idx*3+2);


		Eigen::MatrixXd F;
		F = Eigen::MatrixXd::Zero(6, state_mean.rows());
		F(0,0) = 1;
		F(1,1) = 1;
		F(2,2) = 1;
		F(3,3+landmark_idx*3) = 1;
		F(4,3+landmark_idx*3+1) = 1;
		F(5,3+landmark_idx*3+2) = 1;

		Eigen::MatrixXd H_reduced, HRHi;
		H_reduced = Eigen::MatrixXd::Zero(3,6);
		H_reduced(0,0) = -cos(state_mean(2));
		H_reduced(0,1) = -sin(state_mean(2));
		H_reduced(1,0) =  sin(state_mean(2));
		H_reduced(1,1) = -cos(state_mean(2));
		H_reduced(0,2) =  q_y;
		H_reduced(1,2) = -q_x;
		H_reduced(0,3) =  cos(state_mean(2));
		H_reduced(0,4) =  sin(state_mean(2));
		H_reduced(1,3) = -sin(state_mean(2));
		H_reduced(1,4) =  cos(state_mean(2));
		H_reduced(2,5) = 1;

		HRHi = H_reduced*F;
		H_accu.block(3*mm_i, 0, 3, state_mean.rows()) = HRHi;
		R_accu.block(3*mm_i, 3*mm_i, 3, 3) = R;
	}
	//std::cout<<"before inverse"<<std::endl;
	Eigen::MatrixXd S = H_accu*state_cov*H_accu.transpose()+R_accu;
	S = (S + S.transpose()) / 2;
	Eigen::MatrixXd K = state_cov * H_accu.transpose() * S.inverse();
	// std::cout<<"res: "<<std::endl<<measurements - _measurements<<std::endl;
	double S_cond = S.norm() * (S.inverse()).norm();
	if (!(S_cond>0 && S_cond<80)) return;
	// std::cout<<"S: "<<std::endl<<S_cond<<std::endl<<"end of S"<<std::endl;
	state_mean += K*(measurements - _measurements);
    //normalize the orientation range
    if(state_mean(2)>=2*PI)
	{
		state_mean(2)=state_mean(2)-2*PI;
	}
	else if(state_mean(2)<0)
	{
		state_mean(2)=2*PI+state_mean(2);
	}
	state_cov = state_cov - K*S*K.transpose();
	state_cov = (state_cov + state_cov.transpose()) / 2;

	geometry_msgs::PoseStamped test_pose;
	ros::Time new_now = ros::Time::now();
	test_pose.header.stamp = ros::Time(new_now.sec, new_now.nsec);
	test_pose.header.frame_id = "/map";
	test_pose.pose.position.x = state_mean(0);
	test_pose.pose.position.y = state_mean(1);
	test_pose.pose.position.z = 0;
	test_pose.pose.orientation.z = sin(state_mean(2)/2);
	test_pose.pose.orientation.w = cos(state_mean(2)/2);
    robot_state_pub.publish(test_pose);

    std::vector<double> current_state = state_to_vector();
	update_history.push_back(current_state);
	if (update_history.size() % 10 == 0) {
		write_to_csv("update.csv", update_history);
	}
}

void match(const Eigen::MatrixXd& srcKeyPoints, const std::vector< boost::dynamic_bitset<> >& srcDescriptors, const Eigen::MatrixXd& destKeyPoints, const std::vector< boost::dynamic_bitset<> >& destDescriptors, std::vector<std::array<size_t, 3> >& matches, std::vector<std::array<size_t, 3> >& new_points, double max_signature_threshold, int match_threshold, int new_points_threshold)
{
	for(size_t i=0;i<srcKeyPoints.cols();i++)
  {
    size_t idx = -1;
    size_t dist = -1;
    size_t scd_dist = -1;
    for (size_t j=0;j<destKeyPoints.cols();j++)
    {
      double signature = srcKeyPoints(2,i) - destKeyPoints(2,j);
      signature = signature>0 ? signature : -signature;
      if(signature > max_signature_threshold)
        continue;

      boost::dynamic_bitset<> cur_bit = srcDescriptors[i] ^ destDescriptors[j];
      size_t cur_dist = cur_bit.count();
      if(dist > cur_dist)
      {
        idx = j;
        scd_dist = dist;
        dist = cur_dist;
      }
    }
    // std::cout<<"dest: "<<destKeyPoints.cols()<<std::endl;
    // std::cout<<"dist: "<<dist<<std::endl;
    if(dist < match_threshold && idx>0)
    {
      std::array<size_t, 3> match_i = {i, idx, dist};
      matches.push_back(match_i);
    }
    else if(dist > new_points_threshold)
    {
      std::array<size_t, 3> np_i = {i, idx, dist};
      new_points.push_back(np_i);
    }
  }
}

void EKF_SLAM::landmark_match(const Eigen::MatrixXd& srcKeyPoints, const std::vector< boost::dynamic_bitset<> >& srcDescriptors, std::vector<std::array<size_t, 3> >& matches, std::vector<std::array<size_t, 3> >& new_points, double max_signature_threshold, int match_threshold, int new_points_threshold) const
{
	Eigen::MatrixXd destKeyPoints(3, num_landmarks);
	for(int i=0; i<num_landmarks; i++)
	{
		destKeyPoints(0, i) = state_mean(3+i*3);
		destKeyPoints(1, i) = state_mean(3+i*3+1);
		destKeyPoints(2, i) = state_mean(3+i*3+2);
	}
	std::vector< boost::dynamic_bitset<> > destDescriptors = descriptorDB;

  std::vector<std::array<size_t, 3>> l_matches;
  std::vector<std::array<size_t, 3>> r_matches;
  std::vector<std::array<size_t, 3>> t_new;

  match(srcKeyPoints, srcDescriptors, destKeyPoints, destDescriptors, l_matches, new_points, max_signature_threshold, match_threshold, new_points_threshold);
  match(destKeyPoints, destDescriptors, srcKeyPoints, srcDescriptors, r_matches, t_new, max_signature_threshold, match_threshold, new_points_threshold);

  // std::cout<<"lft: "<<l_matches.size()<<std::endl;
  for(int i=0; i<l_matches.size(); i++)
  {
    for(int j=0; j<r_matches.size(); j++)
    {
      if(l_matches[i][0] == r_matches[j][1] && l_matches[i][1] == r_matches[j][0])
      {
	    // std::cout<<"dist: "<<l_matches[i][2]<<std::endl;
        matches.push_back(l_matches[i]);
	  }
    }
  }
}

void EKF_SLAM::landmark_pcl_pub()
{
	PointCloud::Ptr msg (new PointCloud);
  	msg->header.frame_id = "/map";
  	msg->height = 1;
  	msg->width = num_landmarks;
  	msg->points.resize(msg->height * msg->width);
  	msg->is_dense = false;
  	for(int i=0; i<num_landmarks; i++)
  	{
	    msg->points[i].x = state_mean(3+3*i);
	    msg->points[i].y = state_mean(3+3*i+1);
	    msg->points[i].z = state_mean(3+3*i+2);
  	}
	pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
	pcl_pub.publish(msg);
}

void EKF_SLAM::print_state()
{
	std::cout<<state_mean<<std::endl;
	std::cout<<state_cov<<std::endl;
}

void EKF_SLAM::landmark_count()
{
	std::cout<<(state_mean.rows()-3)/3<<std::endl;
}

void EKF_SLAM::write_to_csv(std::string filename, std::vector< std::vector<double> > dat)
{
	std::ofstream myfile;
	myfile.open(filename);
	for(size_t  i = 0; i < dat.size(); ++i) {
		for (size_t j = 0; j < 12; ++j) {
			if (j < 11) {
				myfile << dat[i][j] << ',';
			} else {
				myfile << dat[i][j] << '\n';
			}
		}
	}
	myfile.close();
}


std::vector<double> EKF_SLAM::state_to_vector() {
	std::vector<double> rval;
	rval.push_back(state_cov(0,0));
	rval.push_back(state_cov(0,1));
	rval.push_back(state_cov(0,2));
	rval.push_back(state_cov(1,0));
	rval.push_back(state_cov(1,1));
	rval.push_back(state_cov(1,2));
	rval.push_back(state_cov(2,0));
	rval.push_back(state_cov(2,1));
	rval.push_back(state_cov(2,2));
	rval.push_back(state_mean(0));
	rval.push_back(state_mean(1));
	rval.push_back(state_mean(2));
	return rval;
}
