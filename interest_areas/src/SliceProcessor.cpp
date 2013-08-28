#include "SliceProcessor.h"

using namespace cv;
using namespace std;

SliceProcessor::SliceProcessor () {

}

vector< vector<Point> > SliceProcessor::compute_interest_area_contours (vector<Mat> slices,
																																				unsigned int thresh) {
	std::vector< std::vector<cv::Point> > all_contours;
	for (size_t i = 0; i < slices.size(); ++i) {
		Mat denoised = denoise (slices[i]);
		std::vector< std::vector<cv::Point> > contours = compute_contours (denoised);
		for (size_t i = 0; i < contours.size(); ++i) {
			if (contours[i].size() > thresh)
				all_contours.push_back (contours[i]);
		}
	}
	cout << "Found " << all_contours.size() << " contours." << endl;
	return all_contours;
}

Mat SliceProcessor::denoise (Mat slice) {
	width_ = slice.rows;
	height_ = slice.cols;
	Mat out;
	erode (slice, out, Mat());
	dilate (out, out, Mat());
	return out;
}

vector< vector<Point> > SliceProcessor::compute_contours (Mat slice) {
	vector< vector<Point> > contours;
	/// Detect edges using canny
	int thresh = 100;
	//Mat edges;
  //Canny( slice, edges, thresh, thresh*2, 3 );
  /// Find contours
 	vector<Vec4i> hierarchy;
	//findContours( edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	findContours( slice, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
	return contours;
}

Mat SliceProcessor::contours_to_mat (vector< vector<Point> > contours) {
	Mat res = Mat::zeros (width_, height_, CV_8UC1);
	for (size_t i = 0; i < contours.size(); ++i) {
		for (size_t j = 0; j < contours[i].size(); ++j) {
			res.at<unsigned char>(contours[i][j].y, contours[i][j].x) = 255;
		}
	}
	return res;
}
/*
vector<segment> segment_lines_ransac () {

}

unsigned int get_inliers (cv::Mat img, double a, double b) {

}

double distance_to_line (unsigned int x, unsigned int y, double a, double b) {

}
*/

