// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-3-23.
//

#ifndef NEW_PLANNERS_CAPSULESHAPE_H
#define NEW_PLANNERS_CAPSULESHAPE_H

#include <variant>
#include <Eigen/Core>
#include "CylinderShell.h"
#include "SphereShell.h"

using CapsulePoint = std::variant<Eigen::Vector3d, CylinderShellPoint>;

struct CapsulePath : public CurvePath<CapsulePoint> {

	using Section = std::variant<HelixPath, SphereShellPath>;

	[[nodiscard]] CapsulePoint at(double t) const override;
};

class CapsuleShape : public WorkspaceShell<CapsulePoint> {

	WorkspaceSphereShell sphere;
	CylinderShell cylinder;

public:
	CapsuleShape(const Eigen::Vector2d &center, double radius, double cylinderHeight);

public:
	[[nodiscard]] Eigen::Vector3d arm_vector(const CapsulePoint &p) const override;

	[[nodiscard]] CapsulePoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	[[nodiscard]] Eigen::Vector3d surface_point(const CapsulePoint &p) const override;

	[[nodiscard]] std::shared_ptr<ShellPath<CapsulePoint>>
	path_from_to(const CapsulePoint &from, const CapsulePoint &to) const override;

	[[nodiscard]] double path_length(const std::shared_ptr<ShellPath<CapsulePoint>> &path) const override;

};


#endif //NEW_PLANNERS_CAPSULESHAPE_H
