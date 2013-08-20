#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& msg) {
	cout << "Callback" << endl;
	pcl::fromROSMsg	(*msg, *cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("octomap_point_cloud_centers", 10, cloud_cb);

	pcl::visualization::CloudViewer viewer ("Viewer");
	while (!viewer.wasStopped ()) {
		viewer.showCloud (cloud);
		ros::spinOnce ();
	}

  return 0;
}
