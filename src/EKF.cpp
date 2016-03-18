#include "EKF.hpp"
#include <iostream>
EKF::EKF()
{
	state_mean = Eigen::Vector3d::Zero();
	state_cov = Eigen::Matrix3d::Zero();
}

EKF::EKF(Eigen::Vector3d _mean, Eigen::Matrix3d _cov)
{
	state_mean = _mean;
	state_cov = _cov;
}

void EKF::predict(double l, double r)
{
	if(l != r)
	{
		double alpha = (r-l) / WHEEL_WIDTH;
		double R = l*WHEEL_WIDTH / (r-l);
		double delta_x = (R + WHEEL_WIDTH/2)*(sin(state_mean.z()+alpha) - sin(state_mean.z()));
		double delta_y = (R + WHEEL_WIDTH/2)*(-cos(state_mean.z()+alpha) + cos(state_mean.z()));
		double delta_theta = alpha;

		Eigen::Vector3d delta_state;
		delta_state << delta_x, 
									delta_y,
									delta_theta;

		Eigen::Matrix3d G;
		G << 1, 0, (R+WHEEL_WIDTH/2)*(cos(state_mean.z()+alpha) - cos(state_mean.z())),
				 0, 1, (R+WHEEL_WIDTH/2)*(sin(state_mean.z()+alpha) - sin(state_mean.z())),
				 0, 0, 1;

		Eigen::MatrixXd V(3,2);
		V << (r*WHEEL_WIDTH)/(r-l)/(r-l) * (sin(state_mean.z()+alpha) - sin(state_mean.z())) - (l+r)/2/(r-l)*(cos(state_mean.z()+alpha)),
				 (l*WHEEL_WIDTH)/(r-l)/(r-l) * (cos(state_mean.z()+alpha) - cos(state_mean.z())) + (l+r)/2/(r-l)*(cos(state_mean.z()+alpha)),
				 (r*WHEEL_WIDTH)/(r-l)/(r-l) * (-cos(state_mean.z()+alpha) + cos(state_mean.z())) - (l+r)/2/(r-l)*(sin(state_mean.z()+alpha)),
				 (l*WHEEL_WIDTH)/(r-l)/(r-l) * (sin(state_mean.z()+alpha) - sin(state_mean.z())) + (l+r)/2/(r-l)*(sin(state_mean.z()+alpha)),
				 -1/WHEEL_WIDTH,
				 1/WHEEL_WIDTH;

		double sigma_l = (MOTION_FACTOR * l)*(MOTION_FACTOR * l) + (TURN_FACTOR *(r-l))*(TURN_FACTOR *(r-l));
		double sigma_r = (MOTION_FACTOR * r)*(MOTION_FACTOR * r) + (TURN_FACTOR *(r-l))*(TURN_FACTOR *(r-l));

		Eigen::Matrix2d cov_motion;
		cov_motion << sigma_l, 0,
									0, sigma_r;

		state_mean = state_mean + delta_state;
		state_cov = G * state_cov * G.transpose() + V * cov_motion * V.transpose();
	}
	else		//l==r
	{
		state_mean.x() = state_mean.x() + l*cos(state_mean.z());
		state_mean.y() = state_mean.y() + l*sin(state_mean.z());
	}
}

