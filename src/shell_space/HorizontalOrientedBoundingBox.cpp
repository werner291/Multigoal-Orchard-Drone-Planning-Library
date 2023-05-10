// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-5-23.
//

#include "HorizontalOrientedBoundingBox.h"

Eigen::Vector3d HorizontalOrientedBoundingBox::arm_vector(const HOBBPoint &p) const {

	if (p.unit_cube_point.x() == 0) {
		return rotation_tf() * Eigen::Vector3d(1, 0, 0);
	} else if (p.unit_cube_point.x() == 1) {
		return rotation_tf() * Eigen::Vector3d(-1, 0, 0);
	} else if (p.unit_cube_point.y() == 0) {
		return rotation_tf() * Eigen::Vector3d(0, 1, 0);
	} else if (p.unit_cube_point.y() == 1) {
		return rotation_tf() * Eigen::Vector3d(0, -1, 0);
	} else if (p.unit_cube_point.z() == 0) {
		return rotation_tf() * Eigen::Vector3d(0, 0, 1);
	} else if (p.unit_cube_point.z() == 1) {
		return rotation_tf() * Eigen::Vector3d(0, 0, -1);
	}

}

HOBBPoint HorizontalOrientedBoundingBox::nearest_point_on_shell(const Eigen::Vector3d &p) const {


}

Eigen::Vector3d HorizontalOrientedBoundingBox::surface_point(const HOBBPoint &p) const {
	return Eigen::Vector3d();
}

std::shared_ptr<ShellPath<HOBBPoint>>
HorizontalOrientedBoundingBox::path_from_to(const HOBBPoint &from, const HOBBPoint &to) const {
	return std::shared_ptr<ShellPath<HOBBPoint>>();
}

double HorizontalOrientedBoundingBox::path_length(const std::shared_ptr<ShellPath<HOBBPoint>> &path) const {
	return 0;
}

Eigen::AngleAxisd HorizontalOrientedBoundingBox::rotation_tf() const {
	return Eigen::AngleAxisd(rotation, Eigen::Vector3d::UnitZ());
}
