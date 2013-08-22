#include "OctreeProcessor.h"

using namespace std;
using namespace octomap;
using namespace cv;

OctreeProcessor::OctreeProcessor (): cloud(new pcl::PointCloud<pcl::PointXYZ>) {
	tree = new OcTree(1.0); // dummy resolution. Will be overwritten by load_tree.
}

void OctreeProcessor::load_tree (const char* path) {
	tree->readBinary(path);
  maxDepth_ = tree->getTreeDepth ();
  resolution_ = tree->getResolution (); // octree resolution in meters
  cout << "Loaded tree with depth " << maxDepth_
  		 << " and resolution " << resolution_ << endl;
	// Give up the multiresolution, all cells will have 0.05 size
	tree->expand();
	// Compute min and max once and for all
	double x, y, z;
	tree->getMetricMin (x, y, z);
	min_ = point3d(x, y, z);
	tree->getMetricMax (x, y, z);
	max_ = point3d(x, y, z);
	cout << "Min : " << min_ << " "
			 << "Max : " << max_ << endl;
	voxel_grid_size_ = point_to_voxel_coord (max_);
	cout << "Voxel grid size : " << voxel_grid_size_ << endl;
}

vector<Mat>	OctreeProcessor::get_interest_areas () {
	vector<Mat> slices;
	std::vector<unsigned int> hist = compute_histogram_z ();
	std::vector<octomap::point3d> walls = get_walls (hist);
	remove (walls);
	// compute new histogram
	hist = compute_histogram_z ();
	std::vector<unsigned int> maxs = find_max_bins (hist);
	for (size_t i = 0; i < maxs.size(); ++i) {
		std::vector<octomap::point3d> slice = get_slice_z (maxs[i]);
		cv::Mat proj = project_slices_2d (slice);
		slices.push_back(proj);
	}
	cout << "Found " << slices.size () << " slices." << endl;
	for (size_t i = 0; i < slices.size(); ++i) {
		cout << maxs[i]*resolution_ << " ";
	}
	cout << endl;

	return slices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeProcessor::cloud_to_pcl (vector<point3d> cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < cloud.size (); ++i) {
		pcl_cloud->points.push_back (pcl::PointXYZ (cloud[i].x(),
																								cloud[i].y(),
																								cloud[i].z()));
	}
	return pcl_cloud;
}

vector<unsigned int> OctreeProcessor::compute_histogram_z () {
	unsigned int num_bins = (max_.z() - min_.z())/resolution_;
  cout << "Histogram has " << num_bins << " bins." << endl;
	vector<unsigned int> hist (num_bins);
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			double z = it.getCoordinate().z();
			unsigned int bin = floor(z/resolution_);
	 		hist[bin] += 1;
		}
  }
  return hist;
}
std::vector<octomap::point3d> OctreeProcessor::get_whole_cloud () {

	vector<point3d> cloud;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			cloud.push_back (it.getCoordinate());
		}
  }
  cout << "Whole cloud has " << cloud.size() << " points." << endl;
  return cloud;
}

std::vector<octomap::point3d> OctreeProcessor::get_floor (std::vector<unsigned int> hist) {
	// getting the bin number = index in the vector
	unsigned int max_bin = std::max_element(hist.begin(), hist.end()) - hist.begin ();
	vector<point3d> slice_z = get_slice_z (max_bin);
	cout << "Floor bin : " << max_bin << ", with " << hist[max_bin] <<" elements." << endl;
	
	return slice_z;
}

vector<point3d> OctreeProcessor::get_walls (vector<unsigned int> hist) {
	// getting the bin number = index in the vector
	//unsigned int min_bin = std::min_element(hist.begin(), hist.end()) - hist.begin ();
	unsigned int min_bin = hist.size()-1; // just before the ceiling
	cout << "Walls bin : " << min_bin << ", with " << hist[min_bin] <<" elements." << endl;
	
	//vector<point3d> slice_z = get_slice_z (min_bin);
	double min_z = 1.9, max_z = 2.2;
	vector<point3d> slice_z = get_slice_z (min_z, max_z);
	vector<double> x, y;
	for (size_t i = 0; i < slice_z.size(); ++i) {
		x.push_back (slice_z[i].x());
		y.push_back (slice_z[i].y());
	}
	return get_slice_xy (x, y);
}

void OctreeProcessor::remove (vector<point3d> points) {
	cout << "Removing " << points.size () << " points." << endl;
	for (size_t i = 0; i < points.size(); ++i) {
		tree->updateNode (points[i], octomap::logodds(0.0f));
	}
	tree->updateInnerOccupancy ();
}

Mat OctreeProcessor::project_slices_2d (vector<point3d> slice) {
	Mat proj = Mat::zeros (voxel_grid_size_.y(), voxel_grid_size_.x(), CV_8UC1);
	for (size_t i = 0; i < slice.size(); ++i) {
		point3d coord = point_to_voxel_coord (slice[i]);
		proj.at<unsigned char>(coord.y(), coord.x()) = 255;
	}
	return proj;
}

point3d OctreeProcessor::point_to_voxel_coord (point3d point) {
	unsigned int x = (point.x() - min_.x()) / resolution_;
	unsigned int y = (point.y() - min_.y()) / resolution_;
	unsigned int z = (point.z() - min_.z()) / resolution_;
	return point3d(x, y, z);
}

vector<point3d> OctreeProcessor::get_slice_xy (vector<double> x, vector<double> y) {
	vector<point3d> slice;
  for (size_t i=0; i < x.size (); ++i) {
  	for (double z=min_.z()+resolution_/2; z < max_.z(); z+=resolution_) {
  		//cout << x[i] << " " << y[i] << " " << z << endl;
  		OcTreeNode* node = tree->search(x[i], y[i], z);
  		if (node != NULL) {
	  		if (tree->isNodeOccupied(node)) {
	  			/*
	  			for (unsigned int k = 0; k < 5; ++k) {
	  				for (unsigned int l = 0; l < 5; ++l) {
	  					slice.push_back (point3d(x[i]+k*resolution_, y[i]+l*resolution_, z));
	  				}
	  			}
	  			*/
	  			slice.push_back (point3d(x[i], y[i], z));
  			}
  		}
  	}
  }  
  return slice;
}

vector<point3d> OctreeProcessor::get_slice_z (unsigned int bin) {
	double min_z, max_z;
	bin_to_z (bin, min_z, max_z);
	return get_slice_z (min_z, max_z);
	/*
	double min_z, max_z, tmp;
	bin_to_z (bin, min_z, tmp);
	bin_to_z (bin+1, tmp, max_z);
	return get_slice_z (min_z, max_z);
	*/
}

vector<point3d> OctreeProcessor::get_slice_z (double min_z, double max_z) {
	vector<point3d> slice;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (min_z <= pt.z() && pt.z() <= max_z)
				slice.push_back (pt);
		}
  }
  return slice;
}

void OctreeProcessor::bin_to_z (unsigned int bin, double &min_z, double &max_z) {
	min_z = bin * resolution_;
	max_z = (bin+1) * resolution_;
}

bool OctreeProcessor::is_free_above (point3d pt, unsigned int num_free_voxels) {
	for (unsigned int i=1; i < num_free_voxels+1; ++i) {
		//cout << pt.x() << " " << pt.y() << " " << z << endl;
		OcTreeNode* node = tree->search(pt.x(), pt.y(), pt.z() + i*resolution_);
		if (node == NULL || tree->isNodeOccupied(node)) {
			return false;
		}
	}
	return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeProcessor::tree_to_pointcloud () {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate();
			cloud->points.push_back (pcl::PointXYZ (pt.x(), pt.y(), pt.z()));
		}
  }
  return cloud;
}

void OctreeProcessor::pointcloud_to_tree (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

}

vector<unsigned int> OctreeProcessor::find_max_bins (vector<unsigned int> hist) {
	vector<unsigned int> maxs;
	for (size_t i = 1; i < hist.size()-1; ++i) {
		if ( (hist[i-1] < hist[i]) && hist[i] > hist[i+1])
			maxs.push_back (i);
	}
	
	cout << "Maxs size : " << maxs.size() << endl;
	for (size_t i = 0; i < maxs.size (); ++i) {
		cout << maxs[i] << " ";
	}
	cout << endl;
	return maxs;
}

vector<point3d> OctreeProcessor::get_open_areas (unsigned int num_free_voxels) {
	vector<point3d> points;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (is_free_above(pt, num_free_voxels)) {
				points.push_back (pt);
			}
		}
  }
  cout << "Found " << points.size () << " points out there in the open." << endl;
  return points;
}

vector<point3d> OctreeProcessor::get_non_open_areas (unsigned int num_free_voxels) {
	vector<point3d> points;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (!is_free_above(pt, num_free_voxels)) {
				points.push_back (pt);
			}
		}
  }
  cout << "Found " << points.size () << " points out there in the open." << endl;
  return points;
}
