// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-5-23.
//

#include "HorizontalOrientedBoundingBox.h"

Eigen::AngleAxisd HorizontalOrientedBoundingBox::rotation_tf() const {
	return Eigen::AngleAxisd(rotation, Eigen::Vector3d::UnitZ());
}

Eigen::Vector3d HorizontalOrientedBoundingBox::unit_cube_to_bounding_box(const Eigen::Vector3d &unit_cube_point) const {
	return center + rotation_tf() * (unit_cube_point.cwiseProduct(half_extents));
}

Eigen::Vector3d HorizontalOrientedBoundingBox::arm_vector(const HOBBPoint &p) const {

	// Find the biggest component of the unit cube point

	if (p.unit_cube_point.x() == 1.0) {
		return rotation_tf() * -Eigen::Vector3d::UnitX();
	} else if (p.unit_cube_point.x() == -1.0) {
		return rotation_tf() * Eigen::Vector3d::UnitX();
	} else if (p.unit_cube_point.y() == 1.0) {
		return rotation_tf() * -Eigen::Vector3d::UnitY();
	} else if (p.unit_cube_point.y() == -1.0) {
		return rotation_tf() * Eigen::Vector3d::UnitY();
	} else if (p.unit_cube_point.z() == 1.0) {
		return rotation_tf() * -Eigen::Vector3d::UnitZ();
	} else {
		return rotation_tf() * Eigen::Vector3d::UnitZ();
	}


}

HOBBPoint HorizontalOrientedBoundingBox::nearest_point_on_shell(const Eigen::Vector3d &p) const {
	Eigen::Vector3d local_point = rotation_tf().inverse() * (p - center);
	Eigen::Vector3d clamped_point = local_point.cwiseQuotient(half_extents).cwiseMax(-Eigen::Vector3d::Ones()).cwiseMin(Eigen::Vector3d::Ones());

	// Find the face closest to the clamped_point, taking the half-extent of the box into account
	double max_ratio = -1.0;
	int max_ratio_index = -1;
	for (int i = 0; i < 3; ++i) {
		double ratio = std::abs(clamped_point(i));
		if (ratio > max_ratio) {
			max_ratio = ratio;
			max_ratio_index = i;
		}
	}

	// Set the clamped_point to be on the surface of the unit cube
	clamped_point(max_ratio_index) = std::copysign(1.0, clamped_point(max_ratio_index));

	return HOBBPoint{clamped_point};
}

Eigen::Vector3d HorizontalOrientedBoundingBox::surface_point(const HOBBPoint &p) const {
	return unit_cube_to_bounding_box(p.unit_cube_point);
}

std::shared_ptr<ShellPath<HOBBPoint>>
HorizontalOrientedBoundingBox::path_from_to(const HOBBPoint &from, const HOBBPoint &to) const {
	std::vector<HOBBPoint> points;
	points.push_back(from);

	Eigen::Vector3d current_point = from.unit_cube_point;
	Eigen::Vector3d direction = (to.unit_cube_point - from.unit_cube_point).normalized();

	while ((current_point - to.unit_cube_point).norm() > 1e-6) {

		// If the current point and the destination point are on the same face, we can just add the destination point
		// and break out of the loop

		if (current_point.x() == to.unit_cube_point.x() && std::abs(current_point.x()) == 0.0) {
			points.push_back(to);
			break;
		} else if (current_point.y() == to.unit_cube_point.y() && std::abs(current_point.y()) == 0.0) {
			points.push_back(to);
			break;
		} else if (current_point.z() == to.unit_cube_point.z() && std::abs(current_point.z()) == 0.0) {
			points.push_back(to);
			break;
		}

		// Else, we are not on the same face.

		// Find the face index and axis of the current point
		int axis = -1;
		for (int i = 0; i < 3; ++i) {
			if (std::abs(std::abs(current_point[i]) - 1.0) < 1e-6) {
				axis = i;
				break;
			}
		}
		assert(axis != -1 && "Current point is not on a face of the unit cube.");


		// Project the direction vector onto the current face
		Eigen::Vector3d face_direction = direction;
		face_direction[axis] = 0.0;

		// Calculate the intersection with the edge of the cube or the destination point
		double min_t = 1.0 / face_direction.norm();
		double t_to_dest = (to.unit_cube_point - current_point).dot(face_direction) / face_direction.squaredNorm();

		if (t_to_dest >= 0.0 && t_to_dest <= min_t) {
			current_point = to.unit_cube_point;
		} else {
			Eigen::Vector3d delta = face_direction * min_t;
			current_point += delta;
		}
	}

	// Create and return a PiecewiseLinearPath object containing the calculated path points
	return std::make_shared<PiecewiseLinearPath<HOBBPoint>>(points);
}

double HorizontalOrientedBoundingBox::path_length(const std::shared_ptr<ShellPath<HOBBPoint>> &path) const {
	auto piecewise_linear_path = std::dynamic_pointer_cast<PiecewiseLinearPath<HOBBPoint>>(path);
	if (!piecewise_linear_path) {
		throw std::runtime_error("Unsupported path type.");
	}

	double length = 0.0;
	for (size_t i = 1; i < piecewise_linear_path->points.size(); ++i) {
		length += (surface_point(piecewise_linear_path->points[i]) - surface_point(piecewise_linear_path->points[i - 1])).norm();
	}
	return length;
}

HorizontalOrientedBoundingBox::HorizontalOrientedBoundingBox(const Eigen::Vector3d &center,
															 const Eigen::Vector3d &halfExtents,
															 double rotation)
		: center(center), half_extents(halfExtents), rotation(rotation) {

	assert(
			halfExtents.x() > 0.0 &&
			halfExtents.y() > 0.0 &&
			halfExtents.z() > 0.0
			);

}

HOBBPoint::HOBBPoint(const Eigen::Vector3d &unitCubePoint) : unit_cube_point(unitCubePoint) {
	assert(
			std::abs(unitCubePoint.x()) == 1.0 ||
			std::abs(unitCubePoint.y()) == 1.0 ||
			std::abs(unitCubePoint.z()) == 1.0
			);
}
