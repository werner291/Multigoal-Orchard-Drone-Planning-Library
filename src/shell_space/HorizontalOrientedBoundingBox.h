// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-5-23.
//

#ifndef NEW_PLANNERS_HORIZONTALORIENTEDBOUNDINGBOX_H
#define NEW_PLANNERS_HORIZONTALORIENTEDBOUNDINGBOX_H

#include <Eigen/Geometry>
#include "WorkspaceShell.h"

struct HOBBPoint {
	Eigen::Vector3d unit_cube_point; // A point on the unit cube
};

class HorizontalOrientedBoundingBox : public WorkspaceShell<HOBBPoint> {

	Eigen::Vector3d center;
	Eigen::Vector3d half_extents;
	double rotation;

	Eigen::AngleAxisd rotation_tf() const;

public:
	[[nodiscard]] Eigen::Vector3d arm_vector(const HOBBPoint &p) const override;

	[[nodiscard]] HOBBPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	[[nodiscard]] Eigen::Vector3d surface_point(const HOBBPoint &p) const override;

	[[nodiscard]] std::shared_ptr<ShellPath<HOBBPoint>>
	path_from_to(const HOBBPoint &from, const HOBBPoint &to) const override;

	[[nodiscard]] double path_length(const std::shared_ptr<ShellPath<HOBBPoint>> &path) const override;

};


#endif //NEW_PLANNERS_HORIZONTALORIENTEDBOUNDINGBOX_H
