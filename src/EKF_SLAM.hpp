#include "ros/ros.h"
#include <vector>
#include <fstream>
#include <Eigen/LU>
#include <Eigen/Core>
#include <math.h>
#include <boost/dynamic_bitset.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#define KINECT_DISP 0.07	//measured
#define KINECT_X_VAR 0.7
#define KINECT_Y_VAR 0.7
#define KINECT_S_VAR 0.7
#define MOTION_FACTOR 1
#define TURN_FACTOR 1
#define PI 3.1415926

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class EKF_SLAM
{
private:
	Eigen::VectorXd state_mean;
	Eigen::MatrixXd state_cov;

	// store robot state estimates for matlab trajectory evaluation
	std::vector< std::vector<double> > propagation_history;
	std::vector< std::vector<double> > update_history;

	Eigen::Matrix3d G_accu;
	Eigen::Matrix3d R;
	bool accu_flag;
	int num_landmarks;
	double delta_t; // define according to clock rate
	std::vector< boost::dynamic_bitset<> > descriptorDB;

	ros::NodeHandle nh;
	ros::Publisher robot_state_pub;
	ros::Publisher pcl_pub;

public:
	EKF_SLAM();
	EKF_SLAM(Eigen::Vector3d _mean, Eigen::Matrix3d _cov);
	void predict(double l, double r, double t);
	void add_landmark(double x, double y, double sig, boost::dynamic_bitset<> dscrt);
	void measurement_update(Eigen::VectorXd measurement, Eigen::VectorXd measurement_idx);
	void landmark_match(const Eigen::MatrixXd& srcKeyPoints, const std::vector< boost::dynamic_bitset<> >& srcDescriptors, std::vector<std::array<size_t, 3> >& matches, std::vector<std::array<size_t, 3> >& new_points, double max_signature_threshold, double match_threshold, double new_points_threshold) const;
	void landmark_pcl_pub();
	void print_state();
	void landmark_count();
	void write_to_csv(std::string filename, std::vector< std::vector<double> > dat);
	std::vector<double> state_to_vector();
};
