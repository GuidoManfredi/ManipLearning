#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

#include "OrientedBoundingBox.h"
#include "Reco.h"

#include "reco_3d/OrientedBoundingBoxRecognition.h"

using namespace std;
// obb = oriented bounding box
OrientedBoundingBox obb_computer;
Reco reco;

// Callback for pointcloud subscriber
bool recognize (reco_3d::OrientedBoundingBoxRecognition::Request &req,
								reco_3d::OrientedBoundingBoxRecognition::Response &res) {
	Eigen::Vector3f t, dims;
	Eigen::Quaternionf q;

	Cloud::Ptr cloud (new Cloud);
	pcl::fromROSMsg(req.cluster, *cloud);
	obb_computer.compute_obb_pca_hull (cloud, q, t, dims);
	res.pose.header = cloud->header;
	res.pose.pose.position.x = t(0);
	res.pose.pose.position.y = t(1);
	res.pose.pose.position.z = t(2);
	res.pose.pose.orientation.x = q.x();
	res.pose.pose.orientation.y = q.y();
	res.pose.pose.orientation.z = q.z();
	res.pose.pose.orientation.w = q.w();
	res.box_dims.x = dims.x; res.box_dims.y = dims.y; res.box_dims.z = dims.z;

	res.names = reco.recognize (w, h, d, 0.99, req.names);
	res.result = 0;
	
	return true;
}

int main(int argc, char **argv) {
	if (argc != 2) {
		cout << "Usage : reco_3d_node stats_filepath" << endl;
		return 1;
	}

  ros::init(argc, argv, "reco_3d_node");
  ros::NodeHandle n;

  ros::ServiceServer srv_plans = n.advertiseService("/recognition_obb", recognize);
  
  reco.load_training_file (argv[1]);
  
  cout << "Recognition oriented bounding boxes service ready." << endl;
 	ros::Rate loop_rate(10);
  while (ros::ok()) {
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
