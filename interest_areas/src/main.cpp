#include "OctreeProcessor.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


int main (int argc, char const* argv[])
{
	OctreeProcessor op;
	op.load_tree (argv[1]);
	//std::vector<unsigned int> hist = op.compute_histogram ();
	//std::vector<octomap::point3d> walls = op.get_walls (hist);
	std::vector<octomap::point3d> planes = op.get_open_areas (10);

	pcl::visualization::CloudViewer viewer ("Viewer");
	//viewer.showCloud (op.cloud_to_pcl (walls));
	viewer.showCloud (op.cloud_to_pcl (planes));
	while (!viewer.wasStopped ()) {
	}
  
	return 0;
}
