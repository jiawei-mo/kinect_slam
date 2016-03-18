#include <Eigen/Dense>
#include <math.h> 

#define WHEEL_WIDTH 0.5
#define MOTION_FACTOR 0.3
#define TURN_FACTOR 0.65

class EKF
{
private: 
	Eigen::Vector3d state_mean;
	Eigen::Matrix3d state_cov;

public:
	EKF();
	EKF(Eigen::Vector3d _mean, Eigen::Matrix3d _cov);
	void predict(double l, double r);
};