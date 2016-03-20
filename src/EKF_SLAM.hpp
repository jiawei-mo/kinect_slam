#include <Eigen/Dense>
#include <math.h> 
#include <boost/dynamic_bitset.hpp>
#include "opencv2/features2d/features2d.hpp"

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
	std::vector< boost::dynamic_bitset<> > descriptorDB;

public:
	EKF_SLAM();
	EKF_SLAM(Eigen::Vector3d _mean, Eigen::Matrix3d _cov);
	void predict(double l, double r);
	void add_landmark(double x, double y, double z, boost::dynamic_bitset<> dscrt);
	void measurement_update(Eigen::Vector3d measurement, int landmark_idx);
	void landmark_match(const cv::Mat& srcKeyPoints, const Eigen::MatrixXd& destKeyPoints, const std::vector< boost::dynamic_bitset<> >& srcDescriptors, const std::vector< boost::dynamic_bitset<> >& destDescriptors, std::vector<cv::DMatch>& matches, double vertical_offset, double max_horizontal_threshold, double min_horizontal_threshold, double dist_threshold) const;
	void getMeasurement(Eigen::MatrixXd& msmtDB, std::vector< boost::dynamic_bitset<> >& dscrtDB);
	void print_state();
};