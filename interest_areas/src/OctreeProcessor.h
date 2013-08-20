#pragma once
// octomap
#include <octomap/octomap.h>
// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class OctreeProcessor
{
	public:
		OctreeProcessor ();
		void load_tree (const char* path);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_pcl (std::vector<octomap::point3d> cloud);
		std::vector<unsigned int> compute_histogram ();

		std::vector<octomap::point3d> get_walls (std::vector<unsigned int> hist);
		std::vector<octomap::point3d> get_open_areas (unsigned int num_free_voxels);

		std::vector<octomap::point3d> get_slice_xy (std::vector<double> x, std::vector<double> y);
		std::vector<octomap::point3d> get_slice_z (unsigned int bin);
		std::vector<octomap::point3d> get_slice_z (double min_z, double max_z);

	private:
		void bin_to_z (unsigned int bin, double &min_z, double &max_z);
		bool is_free_above (octomap::point3d pt, unsigned int num_free_voxels);
	
		octomap::OcTree* tree;
		double resolution_;
		unsigned int maxDepth_;
		double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};
