#include "EKF.hpp"
#include <iostream>
EKF::EKF()
{
	state_mean = Eigen::Vector3d::Zero();
	state_cov = Eigen::Matrix3d::Zero();
	num_landmarks = 0;
}

EKF::EKF(Eigen::Vector3d _mean, Eigen::Matrix3d _cov)
{
	state_mean = _mean;
	state_cov = _cov;
	num_landmarks = 0;
}

void EKF::predict(double l, double r)
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
				 -1/WHEEL_WIDTH,
				 1/WHEEL_WIDTH;
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
	std::cout<<state_mean<<std::endl;
	std::cout<<state_cov<<std::endl;
}
