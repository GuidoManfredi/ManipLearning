#pragma once

#include <octomap/octomap.h>

octomap::point3d point_to_voxel_coord (octomap::point3d point,
																											  double resolution,
																											  octomap::point3d min);

octomap::point3d voxel_coord_to_point (octomap::point3d coord,
																												double resolution,
																												octomap::point3d min);

void bin_to_z (unsigned int bin, double resolution,
																double &min_z, double &max_z);
