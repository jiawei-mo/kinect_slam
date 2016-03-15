#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <boost/dynamic_bitset.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

static const std::string OPENCV_WINDOW = "Image Window";
class StereoMatcher
{
private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	float dist_threshold;
	double tf_threshold;

public:
	StereoMatcher(float d_th, double tf_th);
	~StereoMatcher();
	void disparity_match(const std::vector<cv::KeyPoint>& srcKeyPoints, const std::vector<cv::KeyPoint>& destKeyPoints, const std::vector< boost::dynamic_bitset<> >& srcDescriptors, const std::vector< boost::dynamic_bitset<> >& destDescriptors, std::vector<cv::DMatch>& matches, int vertical_offset, int max_disparity_threshold, int min_disparity_threshold) const;
	void getTF(const cv::Mat& last, const cv::Mat& cur, cv::Mat& R, cv::Mat& T, int ransac_n);
	void drawDisparity(const cv::Mat& src_img, const std::vector<cv::KeyPoint>& src_kp, const std::vector<cv::KeyPoint>& dst_kp, const std::vector<cv::DMatch>& matches);
};