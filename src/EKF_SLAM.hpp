#include "ros/ros.h"
#include <Eigen/LU>
#include <Eigen/Core>
#include <math.h>
#include <boost/dynamic_bitset.hpp>
//#include <geometry_msgs/Pose2D.h>
#include "kinect_slam/Pose2DMsg.h"
#define KINECT_DISP 0.07	//measured
#define KINECT_X_VAR 0.7
#define KINECT_Y_VAR 0.7
#define KINECT_S_VAR 0.7
#define MOTION_FACTOR 1
#define TURN_FACTOR 1
#define PI 3.1415926

class EKF_SLAM
{
private:
	Eigen::VectorXd state_mean;
	Eigen::MatrixXd state_cov;

	Eigen::Matrix3d G_accu;
	Eigen::Matrix3d R;
	bool accu_flag;
	int num_landmarks;
	double delta_t; // define according to clock rate
	std::vector< boost::dynamic_bitset<> > descriptorDB;

	ros::NodeHandle nh;
	ros::Publisher robot_state_pub;

public:
	EKF_SLAM();
	EKF_SLAM(Eigen::Vector3d _mean, Eigen::Matrix3d _cov);
	void predict(double l, double r, double t);
	void add_landmark(double x, double y, double sig, boost::dynamic_bitset<> dscrt);
	void measurement_update(Eigen::VectorXd measurement, Eigen::VectorXd measurement_idx);
	void landmark_match(const Eigen::MatrixXd& srcKeyPoints, const std::vector< boost::dynamic_bitset<> >& srcDescriptors, std::vector<std::array<size_t, 3> >& matches, double max_signature_threshold, double match_threshold) const;
	void print_state();
	void landmark_count();
};
