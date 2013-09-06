#include "tools.h"

octomap::point3d point_to_voxel_coord (octomap::point3d point,
																											  double resolution,
																											  octomap::point3d min) {
	unsigned int x = (point.x() - min.x()) / resolution;
	unsigned int y = (point.y() - min.y()) / resolution;
	unsigned int z = (point.z() - min.z()) / resolution;
	return octomap::point3d(x, y, z);
}

octomap::point3d voxel_coord_to_point (octomap::point3d coord,
																												double resolution,
																												octomap::point3d min) {
	double x = coord.x() * resolution + min.x() ;
	double y = coord.y() * resolution + min.y() ;
	double z = coord.z() * resolution + min.z() ;
	return octomap::point3d(x, y, z);
}

void bin_to_z (unsigned int bin, double resolution,
																double &min_z, double &max_z) {
	min_z = bin * resolution;
	max_z = (bin+1) * resolution;
}
