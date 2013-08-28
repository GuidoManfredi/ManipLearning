#pragma once
// octomap
#include <octomap/octomap.h>
//#include <octomap/OcTreeLUT.h>
// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OpenCV
#include <opencv2/core/core.hpp>

#include "tools.h"

class OctreeProcessor
{
	public:
		OctreeProcessor ();
		void load_tree (const char* path);
		std::vector<cv::Mat>	get_interest_areas (void);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_pcl (std::vector<octomap::point3d> cloud);


		std::vector<unsigned int> compute_histogram_z ();
		void remove (std::vector<octomap::point3d> points);

		std::vector<octomap::point3d> get_whole_cloud ();
		std::vector<octomap::point3d> get_floor (std::vector<unsigned int> hist);
		std::vector<octomap::point3d> get_walls (std::vector<unsigned int> hist);
		std::vector<octomap::point3d> get_open_areas (unsigned int num_free_voxels);
		std::vector<octomap::point3d> get_non_open_areas (unsigned int num_free_voxels);

		std::vector<octomap::point3d> get_slice_xy (std::vector<double> x, std::vector<double> y);
		std::vector<octomap::point3d> get_slice_z (unsigned int bin);
		std::vector<octomap::point3d> get_slice_z (double min_z, double max_z);
		
		unsigned int num_occupied_above (std::vector<octomap::point3d> slice);
		unsigned int num_free_above (std::vector<octomap::point3d> slice);

		pcl::PointCloud<pcl::PointXYZ>::Ptr tree_to_pointcloud ();
		void pointcloud_to_tree (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr slices_to_pointcloud (std::vector<cv::Mat> slices);

		std::vector<unsigned int> find_max_bins (std::vector<unsigned int> hist);

		cv::Mat project_slices_2d (std::vector<octomap::point3d> slice);
		cv::Mat get_slice_2d (double z);
		std::vector<cv::Mat> get_slice_2d_all ();
		
	private:
		bool is_free_above (octomap::point3d pt, unsigned int num_free_voxels);
	
		octomap::OcTree* tree;
		double resolution_;
		unsigned int maxDepth_;
		octomap::point3d min_, max_;
		octomap::point3d voxel_grid_size_;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};
