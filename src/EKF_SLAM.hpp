#include <Eigen/Dense>
#include <math.h> 

#define KINECT_DISP 0.7
#define KINECT_RANGE_VAR 0.7	//squared
#define KINECT_BARING_VAR 0.7
#define KINECT_DEPTH_VAR 0.7
#define WHEEL_WIDTH 0.5
#define MOTION_FACTOR 0.3
#define TURN_FACTOR 0.65
#define INFINITE_COV 99999999999.0

class EKF_SLAM
{
private: 
	Eigen::VectorXd state_mean;
	Eigen::MatrixXd state_cov;

	int num_landmarks;

public:
	EKF_SLAM();
	EKF_SLAM(Eigen::Vector3d _mean, Eigen::Matrix3d _cov);
	void print_state();
	void predict(double l, double r);
	void add_landmark(double x, double y, double z);
	void measurement_update(Eigen::Vector3d measurement, int landmark_idx);
};