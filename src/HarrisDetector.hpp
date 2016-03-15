#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

class HarrisDetector
{
private:
    double harris_response_threshold;
    int harris_window_size;
    int harris_number_of_points;
    int harris_anms_radius;
    bool harris_fix_number_flag;
    bool harris_anms_flag;
    int harris_blur_size;
    int harris_blur_variance;

public:
	HarrisDetector(int hws, float hrt, int hnp, int har, bool hff, bool haf, int hbs, int hbv);
	~HarrisDetector();
	void extract_fixed_number(std::vector<cv::KeyPoint>& input, std::vector<cv::KeyPoint>& output) const;
	void extract_threshold(std::vector<cv::KeyPoint>& input, std::vector<cv::KeyPoint>& output) const;
	void extract_ANMS(std::vector<cv::KeyPoint>& input, std::vector<cv::KeyPoint>& output, int row_number, int col_number) const;
	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints) const;

protected:
	void harrisCorners(const cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints) const;
};