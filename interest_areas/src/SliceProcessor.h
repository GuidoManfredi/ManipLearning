#pragma once
#include <iostream>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class SliceProcessor
{
	public:
		SliceProcessor ();
		std::vector< std::vector<cv::Point> > compute_interest_area_contours (std::vector<cv::Mat> slices,
																																					unsigned int thresh);
		cv::Mat denoise (cv::Mat slice);
		std::vector< std::vector<cv::Point> > compute_contours (cv::Mat slice);
		cv::Mat contours_to_mat (std::vector< std::vector<cv::Point> > contours);

	private:
		unsigned int width_, height_;
};
