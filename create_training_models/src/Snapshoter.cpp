#include "Snapshoter.h"

using namespace std;
using namespace pcl;

Snapshoter::Snapshoter () : snapshot_ (new pcl::PointCloud<pcl::PointXYZ>){
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setMaxIterations (100);
  seg_.setDistanceThreshold (0.02);
  
  ec_.setClusterTolerance (0.02); // 2cm
  ec_.setMinClusterSize (100);
  ec_.setMaxClusterSize (50000);
}

void Snapshoter::take (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	if (cloud->empty())
		cout << "Warning : took empty snapshot !" << endl;
	*snapshot_ = *cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Snapshoter::clean () {
	cout << "Removing NaN and Inf..." << endl;
	snapshot_ = remove_nan_inf (snapshot_);
	cout << "Removing main plan..." << endl;
	snapshot_ = remove_main_plan (snapshot_);
	cout << "Segmenting clusters..." << endl;
	clusters_ = segment_clusters (snapshot_);
	return snapshot_;
}

bool Snapshoter::save (std::string save_path) {
	//if ( object_->empty() )
		//cout << "Warning : Snapshoter::save : trying to save empty object." << endl;
	cout << "Saving to " << save_path << endl;
	if (pcl::io::savePCDFileASCII(save_path, *clusters_[0]) )
		return true;
	else
		return false;
}

void Snapshoter::clear () {
	snapshot_->clear ();
	//object_->clear ();
}
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr
	Snapshoter::remove_nan_inf (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
	vector<int> indices;
	pcl::removeNaNFromPointCloud (*cloud, *result, indices);
	return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
	Snapshoter::remove_main_plan (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
  // Create the segmentation object for the planar model and set all the parameters
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

	// Segment the largest planar component from the cloud
  seg_.setInputCloud (cloud);
  seg_.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }
  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*result);
  return result;
}

vector<PointCloud<PointXYZ>::Ptr>	Snapshoter::segment_clusters (PointCloud<PointXYZ>::ConstPtr cloud) {
	vector<PointCloud<PointXYZ>::Ptr> clusters;
	// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  ec_.setSearchMethod (tree);
  ec_.setInputCloud (cloud);
  ec_.extract (cluster_indices);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back (cloud_cluster);
  }
  cout << "Found " << clusters.size () << " clusters." << endl;
  return clusters;
}
/*
PointCloud<PointXYZ>::Ptr Snapshoter::interest_cluster (vector<PointCloud<PointXYZ>::Ptr> clusters) {
	unsigned int max_idx = -1;
	unsigned int max_size = 0;
	for (size_t i = 0; i < clusters.size (); ++i) {
		if (clusters[i]->size() > max_size) {
			max_idx = i;
			max_size = clusters[i]->size();
		}
	}
	return clusters[max_idx];
}
*/
