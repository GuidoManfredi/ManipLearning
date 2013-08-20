#include <iostream>
// stdlib
#include <numeric>
#include <algorithm>
// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL
/*
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
*/
// Octomap
#include <octomap/octomap.h>

using namespace std;
using namespace octomap;

OcTree tree(1); // 1 = dummy resolution, will be overwritten when reading tree file.
unsigned int maxDepth;
double resolution;

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

// for unique comparing, need to sort the lists:
bool sortX(const point3d& lhs, const point3d& rhs) {
  return (lhs.x() < rhs.x());
}

bool sortY(const point3d& lhs, const point3d& rhs) {
  return (lhs.y() < rhs.y());
}

bool sortZ(const point3d& lhs, const point3d& rhs) {
  return (lhs.z() < rhs.z());
}

void load_tree (const char* path) {
	tree.readBinary(path);
  maxDepth = tree.getTreeDepth ();
  resolution = tree.getResolution (); // octree resolution in meters
  cout << "Loaded tree with depth " << maxDepth 
  		 << " and resolution " << resolution << endl;
	// Give up the multiresolution, all cells will have 0.05 size
	tree.expand();
}

void tree_keys () {
  for(OcTree::iterator it = tree.begin(maxDepth), end=tree.end(); it!= end; ++it) {
  	OcTreeKey key;
  	tree.coordToKeyChecked (it.getCoordinate(), key);
  	cout << key[0] << " " << key[1] << " " << key[2] << endl;
  }
}

void tree_to_cloud () {
	/*
	// Set has occupied cells already occupied and with a free cell above them
	unsigned int count_no = 0;
	unsigned int count_yes = 0;
  for(OcTree::iterator it = tree.begin(maxDepth), end=tree.end(); it!= end; ++it) {
  	point3d current_point = it.getCoordinate();
  	point3d above_point = current_point;
  	above_point.z() += resolution;
  	//cout << it.getCoordinate () << endl;
  	//cout << above_point << endl;
  	OcTreeNode * above_node (new OcTreeNode);
  	above_node = tree.search (above_point);
  	if (above_node != NULL	
  			&& tree.isNodeOccupied(*it)
  			&& !tree.isNodeOccupied(*above_node)) {
  		cloud->points.push_back (pcl::PointXYZ (current_point.x(),
  																						current_point.y(),
  																						current_point.z()));
  		++count_yes;
  	}
  	else {
  		++count_no;
  	}
  }
  cout << "Removed " << count_no << "/" << count_yes + count_no << " points" << endl;
  */
}

void get_min_max_z (double &min_z, double &max_z) {
	max_z = 0;
	min_z = 100;
	for(OcTree::iterator it = tree.begin(maxDepth), end=tree.end(); it!= end; ++it) {
		if (tree.isNodeOccupied(*it)) {
			double z = it.getCoordinate().z();
			if (z < min_z)
				min_z = z;
			if (z > max_z)
				max_z = z;
		}
  }
}

void get_mean_stdev (vector<double> v, double &mean, double &stdev) {
	double sum = std::accumulate(v.begin(), v.end(), 0.0);
	mean = sum / v.size();
	
	double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
	stdev = std::sqrt(sq_sum / v.size() - mean * mean);
}

void normalize (vector<double> &v) {
	double mean, stdev;
	get_mean_stdev (v, mean, stdev);
	
	for (size_t i=0; i<v.size(); ++i)
		v[i] = (v[i] - mean)/stdev;
}

void altitude_histogram () {
	double min_z, max_z;
	get_min_max_z (min_z, max_z);
	cout << "Min/Max z :" << min_z << " " << max_z << endl;
	unsigned int num_bins = (max_z - min_z)/resolution + 1;
	vector<unsigned int> hist (num_bins);
	//vector<double> z;
	for(OcTree::iterator it = tree.begin(maxDepth), end=tree.end(); it!= end; ++it) {
		if (tree.isNodeOccupied(*it)) {
			//z.push_back (it.getCoordinate().z());
			double z = it.getCoordinate().z();
			unsigned int bin = floor(z/resolution);
	 		hist[bin] += 1;
		}
  }
  
  /*
	normalize (z);
	double min_normalized_z = *std::min_element(z.begin(), z.end());
	double max_normalized_z = *std::max_element(z.begin(), z.end());
	double step = (max_normalized_z - min_normalized_z)/num_bins;
	cout << min_normalized_z << endl;
	cout << step << endl;
  for (size_t i=0; i<z.size(); ++i) {
  	unsigned int bin = floor((z[i]+fabs(min_normalized_z))/step);
 		hist[bin] += 1;
  }
  */
  
  for (size_t i=0; i<hist.size(); ++i) {
  	cout << hist[i] << endl;
  }
}

void coordinate_to_indices (point3d point, double res,
														unsigned int &i, unsigned int &j, unsigned int &k) {
	i = floor (point.x() / res);
	j = floor (point.y() / res);
	k = floor (point.z() / res);
	cout << point.z() << " " << res << " " << floor(point.y() / res) << endl;
	cout << i << " " << j << " " << k << endl;
}
/*
void tree_to_voxelgrid () {
	point3d min, max;
	double x, y, z;
	tree.getMetricMax (x, y, z);
	max.x() = x; max.y() = y; max.z() = z;
	tree.getMetricMin (x, y, z);
	min.x() = x; min.y() = y; min.z() = z;
	point3d size = max - min;
	size.x() /= resolution;
	size.y() /= resolution;
	size.z() /= resolution;
	cout << size << endl;
	vector < vector < vector <double> > > voxelgrid (size.z());
	for (size_t i = 0; i < voxelgrid.size (); ++i) {
		voxelgrid[i].resize (size.y());
		for (size_t j = 0; j < voxelgrid[i].size (); ++j) {
			voxelgrid[i][j].resize (size.x());
		}
	}
	
	for(OcTree::iterator it = tree.begin(maxDepth), end=tree.end(); it!= end; ++it) {
		if (tree.isNodeOccupied (*it)) {
			point3d pt = it.getCoordinate();
			unsigned int i, j, k;
			coordinate_to_indices (pt, resolution, i, j, k);
			cout << i << " " << j << " " << k << endl;
			voxelgrid[i][j][k] = 1;
		}
	}
}
*/

void tree_to_voxelgrid () {
	vector<point3d> points;
	for(OcTree::iterator it = tree.begin(maxDepth), end=tree.end(); it!= end; ++it) {	
		points.push_back(it.getCoordinate());
  }
  
 	sort (points.begin(), points.end(), sortZ);
 	double curr_z = points[0].z();
 	vector< vector<point3d> > sliceXY;
 	vector<point3d> lineXY;
 	for (size_t i=0; i<points.size(); ++i) {
 		if (curr_z == points[i].z())
 			lineXY.push_back (points[i]);
 		else {
 			curr_z = points[i].z();
 			sliceXY.push_back (lineXY);
 			lineXY.clear ();
 		}
 	}
 	
 	vector< vector< vector<point3d> > > voxelgrid;
 	for (size_t i = 0; i < sliceXY.size(); ++i) {
 		sort (sliceXY[i].begin(), sliceXY[i].end(), sortY);
	 	double curr_y = sliceXY[i][0].y();
	 	vector< vector<point3d> > sliceX;
	 	vector<point3d> lineX;
 		for (size_t j=0; j<sliceXY[i].size(); ++j) {
	 		if (curr_y == sliceXY[i][j].y())
	 			lineX.push_back (sliceXY[i][j]);
	 		else {
	 			curr_y = sliceXY[i][j].y();
	 			sliceX.push_back (lineX);
	 			lineX.clear ();
	 		}
 		}
 		voxelgrid.push_back (sliceX);
 	}
 	
 	for (size_t i = 0; i < voxelgrid.size(); ++i) {
 		for (size_t j = 0; j < voxelgrid[i].size(); ++j) {
			sort(voxelgrid[i][j].begin(), voxelgrid[i][j].end(), sortX);
 		}
 	}
	
	vector< vector< vector<unsigned int> > > binary_voxelgrid (voxelgrid.size());
	for (size_t j = 0; j < voxelgrid.size(); ++j) {
		binary_voxelgrid[j].resize (voxelgrid[j].size());
		for (size_t k=0; k<voxelgrid[j].size (); ++k) {
			binary_voxelgrid[j][k].resize (voxelgrid[j][k].size());
		}
	}
	
 	for (size_t i = 0; i < voxelgrid.size(); ++i) {
 		for (size_t j = 0; j < voxelgrid[i].size(); ++j) {
 			cout << voxelgrid[i][j].size () << endl;
 			/*
 			for (size_t k=0; k< voxelgrid[i][j].size (); ++k) {
 				//cout << i << " " << j << " " << k << endl;
 				//cout << tree.isNodeOccupied(tree.search(voxelgrid[i][j][k], 16)) << endl;
		 		if (tree.isNodeOccupied(tree.search(voxelgrid[i][j][k], 16))) {
			 		binary_voxelgrid[i][j][k] = 1;
		 		}
		 		else {
		 			binary_voxelgrid[i][j][k] = 0;
		 		}
		 	}
		 	*/
		}
 	}
 	
 	cout << binary_voxelgrid[0][0][0] << endl;
 	cout << binary_voxelgrid[0][0][1] << endl;
}

int main (int argc, char const* argv[])
{
	load_tree (argv[1]);
	//tree_to_cloud ();
	//tree_keys ();
	//altitude_histogram ();
	tree_to_voxelgrid ();

	/*
	pcl::visualization::CloudViewer viewer ("Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ()) {
	}
	*/
  
	return 0;
}
