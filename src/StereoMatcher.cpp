#include "StereoMatcher.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h> 
#include <pcl_conversions/pcl_conversions.h> 
StereoMatcher::StereoMatcher(float d_th, double tf_th)
{
	dist_threshold = d_th;
	tf_threshold = tf_th;
	pub = nh.advertise<PointCloud>("point2", 1);
}

StereoMatcher::~StereoMatcher()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void StereoMatcher::disparity_match(const std::vector<cv::KeyPoint>& srcKeyPoints, const std::vector<cv::KeyPoint>& destKeyPoints, const std::vector< boost::dynamic_bitset<> >& srcDescriptors, const std::vector< boost::dynamic_bitset<> >& destDescriptors, std::vector<cv::DMatch>& matches, int vertical_offset, int max_horizontal_threshold, int min_horizontal_threshold) const
{
	std::vector<cv::DMatch> l_matches;
	std::vector<cv::DMatch> r_matches;
	for(int i=0;i<srcKeyPoints.size();i++)
	{
		int idx = -1;
		int dsp = -1;
		size_t dist = -1;
		size_t scd_dist = -1;
		for (int j=0;j<destKeyPoints.size();j++)
		{
			int verticalDist = srcKeyPoints[i].pt.y - destKeyPoints[j].pt.y;
			verticalDist = verticalDist>0 ? verticalDist : -verticalDist;
			if(verticalDist > vertical_offset)
				continue;

			int cur_dsp = srcKeyPoints[i].pt.x - destKeyPoints[j].pt.x;
			cur_dsp = cur_dsp>0? cur_dsp : -cur_dsp;
			if(cur_dsp > max_horizontal_threshold || cur_dsp <min_horizontal_threshold)
				continue;

			boost::dynamic_bitset<> cur_bit = srcDescriptors[i] ^ destDescriptors[j];
			size_t cur_dist = cur_bit.count();
			if(dist == -1 || dist > cur_dist)
			{
				idx = j;
				dsp = cur_dsp;
				scd_dist = dist;
				dist = cur_dist;
			}
		}
		if(dist < dist_threshold*scd_dist && idx>0)
		{
			cv::DMatch match_i(i, idx, dsp);
			l_matches.push_back(match_i);
		}
	}

	for(int i=0;i<destKeyPoints.size();i++)
	{
		int idx = -1;
		int dsp = -1;
		size_t dist = -1;
		size_t scd_dist = -1;
		for (int j=0;j<srcKeyPoints.size();j++)
		{
			int verticalDist = destKeyPoints[i].pt.y - srcKeyPoints[j].pt.y;
			verticalDist = verticalDist>0 ? verticalDist : -verticalDist;
			if(verticalDist > vertical_offset)
				continue;

			int cur_dsp = srcKeyPoints[j].pt.x - destKeyPoints[i].pt.x;
			cur_dsp = cur_dsp>0? cur_dsp : -cur_dsp;
			if(cur_dsp > max_horizontal_threshold || cur_dsp <min_horizontal_threshold)
				continue;

			boost::dynamic_bitset<> cur_bit = destDescriptors[i] ^ srcDescriptors[j];
			size_t cur_dist = cur_bit.count();
			if(dist == -1 || dist > cur_dist)
			{
				idx = j;
				dsp = cur_dsp;
				scd_dist = dist;
				dist = cur_dist;
			}
		}
		if(dist < dist_threshold*scd_dist && idx>0)
		{
			cv::DMatch match_i(i, idx, dsp);
			r_matches.push_back(match_i);
		}
	}

	for(int i=0; i<l_matches.size(); i++)
	{
		for(int j=0; j<r_matches.size(); j++)
		{
			if(l_matches[i].queryIdx == r_matches[j].queryIdx || l_matches[i].trainIdx == r_matches[j].trainIdx)
				matches.push_back(l_matches[i]);
		}
	}
}

void StereoMatcher::getTF(const cv::Mat& last, const cv::Mat& cur, cv::Mat& R, cv::Mat& T, int ransac_n)
{
	// cv::Mat last = cv::Mat::eye(4,5,CV_32F);
	// cv::Mat cur = cv::Mat::eye(4,5,CV_32F);
	// cur.row(0) += 1.3;
	// cur.row(1) += 2.4;
	// last.at<float>(3,0) = 1;
	// last.at<float>(3,1) = 1;
	// last.at<float>(3,2) = 1;
	// last.at<float>(3,4) = 1;
	// last.at<float>(0,4) = 1.7;
	// last.at<float>(1,4) = 3.2;
	// cur.at<float>(3,0) = 1;
	// cur.at<float>(3,1) = 1;
	// cur.at<float>(3,2) = 1;
	// cur.at<float>(3,4) = 1;
	// cur.at<float>(0,4) += 1.7;
	// cur.at<float>(1,4) += 3.2;

	cv::Mat _R = cv::Mat::eye(3,3,CV_32F);
	cv::Mat _T = cv::Mat::zeros(3,1,CV_32F);
	cv::Mat min_R = _R;
	cv::Mat min_T = _T;

	if(last.cols > 30)
	{
		srand(time(NULL));
		double min_dis = -1;
		for(int r_n=0; r_n<ransac_n; r_n++)
		{
			int n1 = rand() % last.cols;
			int n2 = n1;
			while (n2 == n1)
				n2 = (n1 + rand()) % last.cols;
			int n3 = n2;
			while (n3 == n2 || n3 == n1)
				n3 = (n2 + rand()) % last.cols;
			int n4 = n3;
			while (n4 == n3 || n4 == n2 || n4 == n1)
				n4 = (n3 + rand()) % last.cols;

			cv::Mat_<float> p_last(3,4), p_cur(3,4);

			cv::Mat last_reduced, cur_reduced;
			last(cv::Range(0,3), cv::Range(0,last.cols)).copyTo(last_reduced);
			cur(cv::Range(0,3), cv::Range(0,cur.cols)).copyTo(cur_reduced);

			last_reduced.col(n1).copyTo(p_last.col(0));
			last_reduced.col(n2).copyTo(p_last.col(1));
			last_reduced.col(n3).copyTo(p_last.col(2));
			last_reduced.col(n4).copyTo(p_last.col(3));
			cur_reduced.col(n1).copyTo(p_cur.col(0));
			cur_reduced.col(n2).copyTo(p_cur.col(1));
			cur_reduced.col(n3).copyTo(p_cur.col(2));
			cur_reduced.col(n4).copyTo(p_cur.col(3));

			// std::cout<<p_last<<std::endl<<p_cur<<std::endl;
			cv::Mat mean_x = cv::Mat::zeros(3,1,CV_32F);
			cv::Mat mean_y = cv::Mat::zeros(3,1,CV_32F);
			for(int i=0; i< 4; i++)
			{
				mean_x += p_last.col(i);
				mean_y += p_cur.col(i);
			}
			mean_x = mean_x / 4;
			mean_y = mean_y / 4;

			float variance_x = 0;
			cv::Mat covariance_xy = cv::Mat::zeros(3,3,CV_32F);
			for(int i=0; i< 4; i++)
			{
			    variance_x += (p_last.at<float>(0,i) - mean_x.at<float>(0,0)) * (p_last.at<float>(0,i) - mean_x.at<float>(0,0));
				variance_x += (p_last.at<float>(1,i) - mean_x.at<float>(1,0)) * (p_last.at<float>(1,i) - mean_x.at<float>(1,0));
				variance_x += (p_last.at<float>(2,i) - mean_x.at<float>(2,0)) * (p_last.at<float>(2,i) - mean_x.at<float>(2,0));
			    covariance_xy += (p_cur.col(i) - mean_y) * (p_last.col(i) - mean_x).t();
			}		
			variance_x = variance_x / 4;
			covariance_xy = covariance_xy / 4;

			cv::Mat_<float> U, D, Vt, S;
			float detSigmaXY;
			
			// std::cout<<covariance_xy<<std::endl;
			cv::SVD::compute(covariance_xy, D, U, Vt);

			S = cv::Mat::eye(3,3,CV_32F);
			detSigmaXY = cv::determinant(covariance_xy);

			if(detSigmaXY == 0)
			{
				continue;
			}

			S.at<float>(2,2) = detSigmaXY<0 ? -1 : 1;

			_R = U * Vt;
			_R = _R.t();
			_T = mean_y- _R*mean_x;
			// std::cout<<_R<<std::endl<<_T<<_c<<std::endl;

			cv::Mat Tf=cv::Mat::eye(4,4,CV_32F);
			for(int i=0; i<3; i++)
			{
				for(int j=0; j<3; j++)
				{
					Tf.at<float>(i,j) = _R.at<float>(i,j);
				}
				Tf.at<float>(i,3) = _T.at<float>(i,0);
			}

			cv::Mat prj_diff = Tf*last - cur;
			//test norm
			double prj_dis = cv::norm(prj_diff); 
			if(min_dis == -1 || min_dis>prj_dis)
			{
				min_dis = prj_dis;
				min_R = _R;
				min_T = _T;
			}
		}
		if(min_dis > tf_threshold)
		{
			min_R = cv::Mat::eye(3,3,CV_32F);
			min_T = cv::Mat::zeros(3,1,CV_32F);
		}
	}
	R = min_R;
	T = min_T;
	// std::cout<<"End"<<std::endl;
}

void StereoMatcher::drawDisparity(const cv::Mat& src_img, const std::vector<cv::KeyPoint>& src_kp, const std::vector<cv::KeyPoint>& dst_kp, const std::vector<cv::DMatch>& matches)
{
	cv::Mat img = src_img;
	for(int i=0; i<matches.size(); i++)
	{
		cv::line(img, src_kp[matches[i].queryIdx].pt, dst_kp[matches[i].trainIdx].pt, CV_RGB(0, 0, 255));
	}
	cv::namedWindow(OPENCV_WINDOW);
	cv::imshow(OPENCV_WINDOW, img);
	cv::waitKey(3);
}