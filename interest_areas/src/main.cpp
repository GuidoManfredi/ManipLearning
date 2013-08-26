#include "OctreeProcessor.h"
#include "SliceProcessor.h"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>

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
	std::vector<unsigned int> hist = op.compute_histogram_z ();
	std::vector<octomap::point3d> walls = op.get_walls (hist);
	//std::vector<octomap::point3d> floor = op.get_floor (hist);
	//std::vector<octomap::point3d> non_open_areas = op.get_non_open_areas (1);
	//std::vector<octomap::point3d> open_areas = op.get_open_areas (1);
	//op.remove (open_areas);
	//op.remove (non_open_areas);
	//op.remove (walls);
	//op.remove (floor);
	//op.remove (non_planes);

	/*
	for (size_t i = 0; i < cloud.size(); ++i) {
		cout << cloud[i].z() << endl;
	}
	*/
	
	std::vector<octomap::point3d> cloud = op.get_whole_cloud ();	
	viewer.showCloud (op.cloud_to_pcl (cloud));
	while (!viewer.wasStopped ()) {}

	//std::vector<cv::Mat> slices = op.get_interest_areas ();
}

void test_slice_processing () {
	std::vector<cv::Mat> slices = op.get_interest_areas ();
	std::vector< std::vector<cv::Point> > contours;
	contours = sp.compute_interest_area_contours (slices, 15);
	cv::Mat final = sp.contours_to_mat (contours);
	//viewer.showCloud (op.cloud_to_pcl (slice));
	cv::namedWindow("Slice");
	cv::imshow ("Slice", final);
	cv::waitKey(0);
}

int main (int argc, char const* argv[])
{
	op.load_tree (argv[1]);
  //test_octree_processing ();
  //test_slice_processing ();
  test_denoise();
	return 0;
}
