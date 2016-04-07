#include "HarrisDetector.hpp"
#include "ros/ros.h"
#include <cmath>

HarrisDetector::HarrisDetector(int hws, float hrt, int hnp, int har, bool hff, bool haf, int hbs, int hbv)
{
	harris_window_size = hws*2+1;
	harris_response_threshold = pow(10, hrt);
	harris_number_of_points = hnp;
	harris_anms_radius = har;
	harris_fix_number_flag = hff;
	harris_anms_flag = haf;
	harris_blur_size = 2*hbs + 1;
	harris_blur_variance = hbv;
}

void HarrisDetector::harrisCorners(const cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints) const
{
	cv::Mat blur_img(image.size(), CV_32F);
	cv::Mat Ix(image.size(), CV_32F), Iy(image.size(), CV_32F);
	cv::Mat Ix2(image.size(), CV_32F), Iy2(image.size(), CV_32F), IxIy(image.size(), CV_32F);
	cv::Mat A(image.size(), CV_32F), B(image.size(), CV_32F), C(image.size(), CV_32F);
	cv::Mat theta_Ix(image.size(), CV_32F), theta_Iy(image.size(), CV_32F);

  	cv::GaussianBlur(image, blur_img, cv::Size(harris_blur_size,harris_blur_size), harris_blur_variance, harris_blur_variance, cv::BORDER_DEFAULT );
  	blur_img.convertTo(blur_img, CV_32F);
  	cv::Sobel( blur_img, Ix, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  	cv::Sobel( blur_img, Iy, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

  	cv::Mat kernel_x, kernel_y, sX, sY;
  	cv::getDerivKernels(sX, sY, 1, 0, harris_window_size, false, CV_32F);
  	kernel_x = sX * sY.t();
  	cv::getDerivKernels(sX, sY, 0, 1, harris_window_size, false, CV_32F);
  	kernel_y = sX * sY.t();
  	cv::filter2D(blur_img, theta_Ix, -1, kernel_x, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
  	cv::filter2D(blur_img, theta_Iy, -1, kernel_y, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);

  	Ix2 = Ix.mul(Ix);
  	Iy2 = Iy.mul(Iy);
  	IxIy = Ix.mul(Iy);

  	float sigma = 0.3*((harris_window_size-1)*0.5-1)+0.8;
  	cv::Mat gaussKernel = cv::getGaussianKernel(harris_window_size, sigma, CV_32F);
  	cv::filter2D(Ix2, A, -1, gaussKernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );
  	cv::filter2D(IxIy, B, -1, gaussKernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );
  	cv::filter2D(Iy2, C, -1, gaussKernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );

  	cv::Mat H(image.size(), CV_32F);
  	H = A.mul(C) - B.mul(B) - 0.04*(A+C).mul(A+C);

  	for(int i=0;i<H.rows;i++)
  	{
  		for(int j=0;j<H.cols;j++)
  		{
  			float& Hij = H.at<float>(i,j);
  			if(Hij>harris_response_threshold)
  			{
  				float theta = std::atan2(theta_Iy.at<float>(i,j), theta_Ix.at<float>(i,j));
  				keyPoints.push_back(cv::KeyPoint(cv::Point(j,i),0,theta,Hij));
  			}
  		}
  	}
}

void HarrisDetector::extract_fixed_number(std::vector<cv::KeyPoint>& input, std::vector<cv::KeyPoint>& output) const
{
	int nop = harris_number_of_points;
	nop = nop>input.size()? input.size() : nop;
	int idx[nop];
	float mag_list[nop];
	for(int i=0;i<nop;i++)
	{
		idx[i] = 0;
		mag_list[i] = 0;
	}

	for(int i=0;i<input.size();i++)
	{
		int j;
		for(j=0;j<nop;j++)
		{
			if(input[i].response < mag_list[j])
			{
				break;
			}
		}

		for(int k=0; k<j-1; k++)
		{
			idx[k] = idx[k+1];
			mag_list[k] = mag_list[k+1];
		}	
		if(j>0)
		{
			idx[j-1] = i;
			mag_list[j-1] = input[i].response;
		}
	}

	for(int i=0;i<nop;i++)
	{
		output.push_back(input[idx[i]]);
	}
}

void HarrisDetector::extract_ANMS(std::vector<cv::KeyPoint>& input, std::vector<cv::KeyPoint>& output, int row_number, int col_number) const
{
  	std::vector<cv::KeyPoint> input_filted;
	extract_fixed_number(input, input_filted);
	cv::Mat max_res = cv::Mat::zeros(row_number, col_number, CV_32F);
	for(int n=0; n<input_filted.size(); n++)
	{
		int row_start = input_filted[n].pt.y - harris_anms_radius < 0 ? 0 : input_filted[n].pt.y - harris_anms_radius;
		int row_end = input_filted[n].pt.y + harris_anms_radius > row_number-1? row_number-1 : input_filted[n].pt.y + harris_anms_radius;
		int col_start = input_filted[n].pt.x - harris_anms_radius < 0 ? 0 : input_filted[n].pt.x - harris_anms_radius;
		int col_end = input_filted[n].pt.x + harris_anms_radius > col_number-1? col_number-1 : input_filted[n].pt.x + harris_anms_radius;
		
		for(int i=row_start; i<row_end; i++)
		{
			for(int j=col_start; j<col_end; j++)
			{
				if(max_res.at<float>(i,j) < input_filted[n].response)
					max_res.at<float>(i,j) = input_filted[n].response;
			}
		}
	}

	for(int n=0; n<input_filted.size(); n++)
	{
		if(input_filted[n].response == max_res.at<float>(input_filted[n].pt.y, input_filted[n].pt.x))
			output.push_back(input_filted[n]);
	}
}

void HarrisDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints) const
{	
  	std::vector<cv::KeyPoint> allKeyPoints;
	harrisCorners(image, allKeyPoints);
  if(harris_anms_flag)
		extract_ANMS(allKeyPoints, keyPoints, image.rows, image.cols);
	else 
		extract_fixed_number(allKeyPoints, keyPoints);

	float max = 0;
	for(int i=0;i<keyPoints.size();i++)
	{
		if(keyPoints[i].response > max)
			max = keyPoints[i].response;
	}

	for(int i=0;i<keyPoints.size();i++)
	{
		keyPoints[i].response = keyPoints[i].response / max;
	}
}
