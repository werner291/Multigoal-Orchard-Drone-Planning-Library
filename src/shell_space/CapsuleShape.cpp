// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-3-23.
//

#include "CapsuleShape.h"

Eigen::Vector3d CapsuleShape::arm_vector(const CapsulePoint &p) const {

	if (std::holds_alternative<Eigen::Vector3d>(p)) {
		return sphere.arm_vector(std::get<Eigen::Vector3d>(p));
	} else {
		return cylinder.arm_vector(std::get<CylinderShellPoint>(p));
	}

}

CapsulePoint CapsuleShape::nearest_point_on_shell(const Eigen::Vector3d &p) const {

	if (sphere.getCenter().z() < p.z()) {
		return sphere.nearest_point_on_shell(p);
	} else {
		return cylinder.nearest_point_on_shell(p);
	}

}

Eigen::Vector3d CapsuleShape::surface_point(const CapsulePoint &p) const {

	if (std::holds_alternative<Eigen::Vector3d>(p)) {
		return sphere.surface_point(std::get<Eigen::Vector3d>(p));
	} else {
		return cylinder.surface_point(std::get<CylinderShellPoint>(p));
	}

}

std::shared_ptr<ShellPath<CapsulePoint>>
CapsuleShape::path_from_to(const CapsulePoint &from, const CapsulePoint &to) const {
	return std::shared_ptr<ShellPath<CapsulePoint>>();
}

double CapsuleShape::path_length(const std::shared_ptr<ShellPath<CapsulePoint>> &path) const {
	return 0;
}

CapsuleShape::CapsuleShape(const Eigen::Vector2d &center, double radius, double cylinderHeight)
		: sphere(Eigen::Vector3d(center.x(), center.y(), cylinderHeight), radius),
		  cylinder(radius, Eigen::Vector2d(center.x(), center.y())) {
}

CapsulePoint CapsulePath::at(double t) const {
	return CapsulePoint();
}
