#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <boost/dynamic_bitset.hpp>
#include <random>

class BRIEF
{
private:
	int brief_size;
	int patch_size;
	int smooth_window_size;
	int **pattern;

public:
	BRIEF(int p, int s, size_t bs)
	{
		int btn = 16;
		for(int i=0; i<bs-1; i++)
			btn *= 2;
		brief_size = btn*8;
		patch_size = p;
		smooth_window_size = s*2+1;

		std::default_random_engine generator;
		std::normal_distribution<float> distribution(0, patch_size*patch_size/25);
		int **_pattern = new int*[brief_size];
		for(int i=0;i<brief_size;i++)
		{
			_pattern[i] = new int[4];
			_pattern[i][0] = distribution(generator);
			_pattern[i][1] = distribution(generator);
			_pattern[i][2] = distribution(generator);
			_pattern[i][3] = distribution(generator);
		}
		pattern = _pattern;
	};
	~BRIEF()
	{
		for(int i=0; i<brief_size; i++)
			delete pattern[i];
		delete pattern;
	};

	void extract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, std::vector< boost::dynamic_bitset<> >& descriptors)
	{
		cv::Mat img;
		cv::GaussianBlur(image, img, cv::Size(smooth_window_size, smooth_window_size), 0, 0, cv::BORDER_DEFAULT);
		img.convertTo(img, CV_16U);

		cv::Mat result = cv::Mat(cv::Size(brief_size, keypoints.size()), CV_32F);
		for(int i=0; i<keypoints.size(); i++)
		{
			boost::dynamic_bitset<> dsct(brief_size);
			for(int j=0; j<brief_size; j++)
			{
				int x = keypoints[i].pt.y;
				int y = keypoints[i].pt.x;
				unsigned short p1 = img.at<unsigned short>(x+pattern[j][0], y+pattern[j][1]);
				unsigned short p2 = img.at<unsigned short>(x+pattern[j][2], y+pattern[j][3]);
				dsct[j] = p1 < p2 ? true : false;
			}
			descriptors.push_back(dsct);
		}
	};
};