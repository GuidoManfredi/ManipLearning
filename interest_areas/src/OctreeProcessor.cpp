#include "OctreeProcessor.h"

using namespace std;
using namespace octomap;

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
	tree->getMetricMin (min_x_, min_y_, min_z_);
	tree->getMetricMax (max_x_, max_y_, max_z_);
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

vector<unsigned int> OctreeProcessor::compute_histogram () {
	unsigned int num_bins = (max_z_ - min_z_)/resolution_;
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

vector<point3d> OctreeProcessor::get_walls (vector<unsigned int> hist) {
	// getting the bin number = index in the vector
	unsigned int min_bin = std::min_element(hist.begin(), hist.end()) - hist.begin ();
	cout << "Minimum bin : " << min_bin << ", with " << hist[min_bin] <<" elements." << endl;
	vector<point3d> slice_z = get_slice_z (min_bin);
	vector<double> x, y;
	for (size_t i = 0; i < slice_z.size(); ++i) {
		x.push_back (slice_z[i].x());
		y.push_back (slice_z[i].y());
	}
	return get_slice_xy (x, y);
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

vector<point3d> OctreeProcessor::get_slice_xy (vector<double> x, vector<double> y) {
	vector<point3d> slice;
  for (size_t i=0; i < x.size (); ++i) {
  	for (double z=min_z_; z < max_z_; z+=resolution_) {
  		//cout << x[i] << " " << y[i] << " " << z << endl;
  		OcTreeNode* node = tree->search(x[i], y[i], z);
  		if (node != NULL) {
	  		if (tree->isNodeOccupied(node)) {
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
}

vector<point3d> OctreeProcessor::get_slice_z (double min_z, double max_z) {
	vector<point3d> slice;
	for(OcTree::iterator it = tree->begin(maxDepth_), end=tree->end(); it!= end; ++it) {
		if (tree->isNodeOccupied(*it)) {
			point3d pt = it.getCoordinate ();
			if (min_z < pt.z() && pt.z() < max_z)
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
		/*
			if (node == NULL)
				cout << "NULL" << endl;
			else
				cout << "Occupied" << endl;
				*/
			return false;
		}
	}
	return true;
}
