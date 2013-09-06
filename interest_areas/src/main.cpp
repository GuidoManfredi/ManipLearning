#include "OctreeProcessor.h"
#include "SliceProcessor.h"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>

// TODO dans octreeprocessor faire un get_min_max_occupied. Permettra de reduire
//	la taille de l'image 2D apres filtrage du bruit dans l'octree.

OctreeProcessor op;
SliceProcessor sp;
pcl::visualization::CloudViewer viewer ("Viewer");

void test_denoise () {
	std::vector<cv::Mat> slices = op.get_slice_2d_all ();
	for (size_t i = 0; i < slices.size(); ++i) {
		sp.denoise (slices[i]);
	}

	viewer.showCloud (op.slices_to_pointcloud (slices));
	while (!viewer.wasStopped ()) {}
}

void test_octree_processing () {
	octomap::point3d min (0.0, 0.0, 0.0), max (10.0, 8.0, 2.2);
	std::vector<octomap::point3d> out_of_bounds = op.get_out_of_bounds (min, max);
	op.remove (out_of_bounds);

	std::vector<unsigned int> hist = op.compute_histogram_z ();
	std::vector<octomap::point3d> floor = op.get_floor (hist);
	op.remove (floor);

	std::vector<octomap::point3d> walls = op.get_walls (hist);
	op.remove (walls);
	
	std::vector<octomap::point3d> noise = op.get_noise ();
	//op.remove (noise);
	//std::vector<octomap::point3d> floor = op.get_floor (hist);
	//std::vector<octomap::point3d> non_open_areas = op.get_non_open_areas (1);
	//std::vector<octomap::point3d> open_areas = op.get_open_areas (1);
	//op.remove (open_areas);
	//op.remove (non_open_areas);
	//op.remove (floor);

	//hist = op.compute_histogram_z ();
	//op.remove (non_planes);
	hist = op.compute_histogram_z ();
	for (size_t i = 0; i < hist.size(); ++i) {
		cout << hist[i] << endl;
	}

	/*
	cv::Mat walls_projected = op.project_slices_2d (walls);
	walls_projected = sp.denoise (walls_projected);
	cv::namedWindow("Slice");
	cv::imshow ("Slice", walls_projected);
	cv::waitKey(0);
	*/
	std::vector<octomap::point3d> cloud = op.get_whole_cloud ();	
	viewer.showCloud (op.cloud_to_pcl (cloud));
	//viewer.showCloud (op.cloud_to_pcl (noise));
	while (!viewer.wasStopped ()) {}

	//std::vector<cv::Mat> slices = op.get_interest_areas ();
}

void test_slice_processing () {
	std::vector<cv::Mat> slices = op.get_interest_areas ();
	std::vector< std::vector<cv::Point> > contours;
	contours = sp.compute_interest_area_contours (slices, 20);
	//viewer.showCloud (op.cloud_to_pcl (slice));
	
	cv::Mat final = sp.contours_to_mat (contours);
	cv::namedWindow("Slice");
	cv::imshow ("Slice", final);
	cv::waitKey(0);
}

void test_interest_clusters (unsigned int i) {
	std::vector< std::vector<octomap::point3d> > clusters;
	clusters = op.get_interest_clouds ();
	
	op.grow_cloud (clusters[i]);
	viewer.showCloud (op.cloud_to_pcl (clusters[i]));
	while (!viewer.wasStopped ()) {}
}

void test_classification () {
	// Denoising
	octomap::point3d min (0.0, 0.0, 0.0), max (10.0, 8.0, 2.2);
	std::vector<octomap::point3d> out_of_bounds = op.get_out_of_bounds (min, max);
	op.remove (out_of_bounds);
	// Get walls
	std::vector<unsigned int> hist = op.compute_histogram_z ();
	std::vector<octomap::point3d> walls = op.get_walls (hist);
	cv::Mat walls_img = op.project_slices_2d (walls);
	// Compute walls positions
	std::vector<cv::Point> inliers;
	cv::Vec4i line = sp.segment_line_ransac (walls_img, 2, 10000, inliers);
	cv::Vec4i line1, line2;
	sp.get_bounding_lines (line, 10, walls_img.cols, walls_img.rows, line1, line2);	
	cout << line << endl << line1 << endl << line2 << endl;
	cv::line (walls_img,  cv::Point(line(0), line(1)),
												cv::Point(line(2), line(3)),
												cv::Scalar(255, 255, 255), 1);
	cv::imwrite ("./test1.png", walls_img);
	cv::line (walls_img,  cv::Point(line1(0), line1(1)),
												cv::Point(line1(2), line1(3)),
												cv::Scalar(255, 255, 255), 1);
	cv::imwrite ("./test2.png", walls_img);
	cv::line (walls_img,  cv::Point(line2(0), line2(1)),
												cv::Point(line2(2), line2(3)),
												cv::Scalar(255, 255, 255), 1);												
	cv::imwrite ("./test3.png", walls_img);
	/*
	std::vector<cv::Point> inliers;
	std::vector<cv::Vec4i> lines = sp.segment_lines (walls_img, inliers);
	cout << "Found " << lines.size() << " segments." << endl;
	cv::Mat display1 = cv::Mat::zeros (walls_img.size(), CV_8UC1);
	for (size_t i = 0; i < lines.size(); ++i) {
		cv::line (display1,  cv::Point(lines[i](0), lines[i](1)),
											  cv::Point(lines[i](2), lines[i](3)),
											  cv::Scalar(255, 255, 255), 1);
	}
	*/
	// Classify walls portions
	//cv::Mat res = op.classify_walls (inliers);
	/*
	//cv::Mat display2 = cv::Mat::zeros (walls_img.size(), CV_8UC1);
	std::vector<cv::Vec4i> new_lines = sp.merge_close_segments (lines);
	for (size_t i = 0; i < new_lines.size(); ++i) {
		cv::line (display2,  cv::Point(new_lines[i](0), new_lines[i](1)),
											  cv::Point(new_lines[i](2), new_lines[i](3)),
											  cv::Scalar(255, 255, 255), 1);
	}
	*/
	cv::namedWindow("Walls");
	cv::imshow ("Walls", walls_img);
	//cv::imshow ("Walls", display1);
	//cv::imshow ("Walls", walls_img);
	//cv::imwrite ("./result_walls.png", display1);
	//cv::imwrite ("./result_new_walls.png", display2);
	//cv::imwrite ("./result_binary.png", walls_img);
	cv::waitKey(0);
}

int main (int argc, char const* argv[])
{
	op.load_tree (argv[1]);
	test_classification ();
	//test_num_occupied ();
  //test_octree_processing ();
  //test_slice_processing ();
  //test_interest_clusters (atoi(argv[2]));
	return 0;
}
