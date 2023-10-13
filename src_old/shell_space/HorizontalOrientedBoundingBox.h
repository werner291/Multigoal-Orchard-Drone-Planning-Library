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

#include "WorkspaceShell.h"

struct HOBBPoint {
	Eigen::Vector3d unit_cube_point;

	explicit HOBBPoint(const Eigen::Vector3d &unitCubePoint);
	// A point on the unit cube
};

/**
 * @brief A workspace shell based on a horizontal oriented bounding box.
 *
 * The shell is defined as a horizontal bounding box, rotated around the Z-axis
 * to minimize the surface area. It uses HOBBPoint for points on the shell.
 */
class HorizontalOrientedBoundingBox : public WorkspaceShell<HOBBPoint> {

	Eigen::Vector3d center; ///< The center of the bounding box
	Eigen::Vector3d half_extents; ///< The half extents of the bounding box
	double rotation; ///< The rotation of the bounding box around the Z-axis

	/**
	 * @brief Get the rotation transform for the bounding box.
	 *
	 * @return The rotation transform as an Eigen::AngleAxisd object.
	 */
	[[nodiscard]] Eigen::AngleAxisd rotation_tf() const;

	/**
	 * @brief Convert a point on the unit cube to a point on the bounding box.
	 *
	 * @param unit_cube_point The point on the unit cube.
	 * @return The corresponding point on the bounding box.
	 */
	[[nodiscard]] Eigen::Vector3d unit_cube_to_bounding_box(const Eigen::Vector3d &unit_cube_point) const;

public:
	HorizontalOrientedBoundingBox(const Eigen::Vector3d &center, const Eigen::Vector3d &halfExtents, double rotation);

	/**
		 * @brief Compute the arm pointing vector for a given HOBBPoint.
		 *
		 * The arm pointing vector is horizontal according to the nearest vertical
		 * side of the unit cube.
		 *
		 * @param p The HOBBPoint on the shell.
		 * @return The arm pointing vector.
		 */
	[[nodiscard]] Eigen::Vector3d arm_vector(const HOBBPoint &p) const override;

	/**
	 * @brief Find the nearest point on the shell to a given point in R^3.
	 *
	 * @param p The point in R^3.
	 * @return The nearest point on the shell.
	 */
	[[nodiscard]] HOBBPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	/**
	 * @brief Find the associated point in R^3 for a given point on the shell.
	 *
	 * @param p The point on the shell.
	 * @return The associated point in R^3.
	 */
	[[nodiscard]] Eigen::Vector3d surface_point(const HOBBPoint &p) const override;

	/**
	 * @brief Return a geodesic path from the given start point to the given end point over the surface of the shell.
	 *
	 * The path is defined by a list of ShellPoints, generated using a piecewise linear path.
	 *
	 * @param from The start point.
	 * @param to The end point.
	 * @return A shared_ptr to the ShellPath object representing the path from the start point to the end point.
	 */
	[[nodiscard]] std::shared_ptr<ShellPath<HOBBPoint>>
	path_from_to(const HOBBPoint &from, const HOBBPoint &to) const override;

	/**
	 * @brief Return the path length for a given ShellPath.
	 *
	 * @param path A shared_ptr to the ShellPath object.
	 * @return The length of the path.
	 */
	[[nodiscard]] double path_length(const std::shared_ptr<ShellPath<HOBBPoint>> &path) const override;

};


#endif //NEW_PLANNERS_HORIZONTALORIENTEDBOUNDINGBOX_H
