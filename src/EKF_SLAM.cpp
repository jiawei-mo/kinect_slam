#include "EKF_SLAM.hpp"
#include <iostream>
EKF_SLAM::EKF_SLAM()
{
	state_mean = Eigen::Vector3d::Zero();
	state_cov = Eigen::Matrix3d::Zero();
	num_landmarks = 0;
}

EKF_SLAM::EKF_SLAM(Eigen::Vector3d _mean, Eigen::Matrix3d _cov)
{
	state_mean = _mean;
	state_cov = _cov;
	num_landmarks = 0;
}

void EKF_SLAM::print_state()
{
	std::cout<<state_mean<<std::endl;
	std::cout<<state_cov<<std::endl;
}

void EKF_SLAM::predict(double l, double r)
{
	double delta_x;
	double delta_y;
	double delta_theta;
	Eigen::Vector3d delta_state;
	Eigen::Matrix3d G;
	Eigen::MatrixXd V(3,2);

	if(l != r)
	{
		double alpha = (r-l) / WHEEL_WIDTH;
		double rad = l*WHEEL_WIDTH / (r-l);
		delta_x = (rad + WHEEL_WIDTH/2)*(sin(state_mean(2)+alpha) - sin(state_mean(2)));
		delta_y = (rad + WHEEL_WIDTH/2)*(-cos(state_mean(2)+alpha) + cos(state_mean(2)));
		delta_theta = alpha;

		G << 1, 0, (rad+WHEEL_WIDTH/2)*(cos(state_mean(2)+alpha) - cos(state_mean(2))),
				 0, 1, (rad+WHEEL_WIDTH/2)*(sin(state_mean(2)+alpha) - sin(state_mean(2))),
				 0, 0, 1;

		V << (r*WHEEL_WIDTH)/(r-l)/(r-l) * (sin(state_mean(2)+alpha) - sin(state_mean(2))) - (l+r)/2/(r-l)*(cos(state_mean(2)+alpha)), (-l*WHEEL_WIDTH)/(r-l)/(r-l) * (sin(state_mean(2)+alpha) - sin(state_mean(2))) + (l+r)/2/(r-l)*(cos(state_mean(2)+alpha)),
				 (r*WHEEL_WIDTH)/(r-l)/(r-l) * (-cos(state_mean(2)+alpha) + cos(state_mean(2))) - (l+r)/2/(r-l)*(sin(state_mean(2)+alpha)), (-l*WHEEL_WIDTH)/(r-l)/(r-l) * (-cos(state_mean(2)+alpha) + cos(state_mean(2))) + (l+r)/2/(r-l)*(sin(state_mean(2)+alpha)),
				 -1/WHEEL_WIDTH, 1/WHEEL_WIDTH;
	}
	else		//l==r
	{
		delta_x = l*cos(state_mean(2));
		delta_y = l*sin(state_mean(2));
		delta_theta = 0;

		G << 1, 0, - l * sin(state_mean(2)),
				 0, 1, l * cos(state_mean(2)),
				 0, 0, 1;

		V << 0.5 * (cos(state_mean(2)) + l/WHEEL_WIDTH*sin(state_mean(2))), 0.5 * (cos(state_mean(2)) - l/WHEEL_WIDTH*sin(state_mean(2))),
				 0.5 * (sin(state_mean(2)) - l/WHEEL_WIDTH*cos(state_mean(2))), 0.5 * (sin(state_mean(2)) + l/WHEEL_WIDTH*cos(state_mean(2))),
				 -1/WHEEL_WIDTH, 1/WHEEL_WIDTH;
	}

	delta_state << delta_x, 
								 delta_y,
								 delta_theta;
	state_mean.block<3,1>(0,0) += delta_state;

	double sigma_l = (MOTION_FACTOR * l)*(MOTION_FACTOR * l) + (TURN_FACTOR *(r-l))*(TURN_FACTOR *(r-l));
	double sigma_r = (MOTION_FACTOR * r)*(MOTION_FACTOR * r) + (TURN_FACTOR *(r-l))*(TURN_FACTOR *(r-l));

	Eigen::Matrix2d cov_control;
	cov_control << sigma_l, 0,
								 0, sigma_r;
	state_cov.block<3,3>(0,0) = G * state_cov.block<3,3>(0,0) * G.transpose() + V * cov_control * V.transpose();
}

void EKF_SLAM::add_landmark(double x, double y, double z)
{
	int former_length = state_mean.rows();

	Eigen::VectorXd state_mean_copy = state_mean;
	state_mean.resize(former_length+3, 1);
	state_mean.block<3,1>(0,0) = state_mean_copy;
	state_mean(former_length) = x;
	state_mean(former_length+1) = y;
	state_mean(former_length+2) = z;

	Eigen::MatrixXd state_cov_copy = state_cov;
	state_cov.resize(former_length+3, former_length+3);
	state_cov.block<3,3>(0,0) = state_cov_copy;
	state_cov(former_length, former_length) = INFINITE_COV;
	state_cov(former_length+1, former_length+1) = INFINITE_COV;
	state_cov(former_length+2, former_length+2) = INFINITE_COV;
}

void EKF_SLAM::measurement_update(Eigen::Vector3d measurement, int landmark_idx)
{
	Eigen::Matrix3d Q;
	Q << KINECT_RANGE_VAR, 0, 0,
			 0, KINECT_BARING_VAR, 0,
			 0, 0, KINECT_DEPTH_VAR;

	double x_kinect = state_mean(0) + KINECT_DISP * cos(state_mean(2));
	double y_kinect = state_mean(1) + KINECT_DISP * sin(state_mean(2));
	double q_x = state_mean(3+landmark_idx*3) - x_kinect;
	double q_y = state_mean(3+landmark_idx*3+1) - y_kinect;
	double q = q_x*q_x + q_y*q_y;
	Eigen::Vector3d _measurement;
	_measurement << sqrt(q),
									atan2(q_y, q_x) - state_mean(2),
									state_mean(3+landmark_idx*3+2);

	Eigen::MatrixXd F;
	F = Eigen::MatrixXd::Zero(6, state_mean.rows());
	F(0,0) = 1;
	F(1,1) = 1;
	F(2,2) = 1;
	F(3+landmark_idx*3,3+landmark_idx*3) = 1;
	F(3+landmark_idx*3+1,3+landmark_idx*3+1) = 1;
	F(3+landmark_idx*3+2,3+landmark_idx*3+2) = 1;

	Eigen::MatrixXd H_reduced, H;
	H_reduced = Eigen::MatrixXd::Zero(3,6);
	H_reduced(0,0) = -sqrt(q)*q_x; 
	H_reduced(0,1) = -sqrt(q)*q_y; 
	H_reduced(0,3) = sqrt(q)*q_x; 
	H_reduced(0,4) = sqrt(q)*q_y; 
	H_reduced(1,0) = q_y; 
	H_reduced(1,1) = -q_x; 
	H_reduced(1,2) = -q; 
	H_reduced(1,3) = -q_y; 
	H_reduced(1,4) = q_x; 
	H_reduced(2,5) = q;
	H_reduced = H_reduced / q;

	H = H_reduced*F;

	Eigen::MatrixXd K = state_cov * H.transpose() * (H*state_cov*H.transpose()+Q).inverse();
	state_mean += K*(measurement - _measurement);
	state_cov = (Eigen::MatrixXd::Identity(state_cov.rows(), state_cov.cols()) - K*H)*state_cov;
}


int main()
{
	EKF_SLAM mySLAM;
	mySLAM.predict(3,3);
	mySLAM.print_state();
	mySLAM.add_landmark(6,3,8);
	mySLAM.print_state();
	Eigen::Vector3d measurement(4.242,0.7854,8);
	mySLAM.measurement_update(measurement, 0);
	mySLAM.print_state();
}