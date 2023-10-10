// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_GRIDAABB_H
#define MGODPL_GRIDAABB_H

#include <Eigen/Geometry>
#include <optional>

namespace mgodpl::math {

	/**
	 * A struct that defines a regular grid of axis-aligned boxes,
	 * based off of a base AABB with some given natural number of
	 * subdivisions in each dimension.
	 */
	class AABBGrid {

		Eigen::AlignedBox3d base_aabb;
		size_t nx, ny, nz;

	public:
		/**
		 * Create a new AABBGrid.
		 * @param base_aabb 	The base AABB.
		 * @param nx 			The number of subdivisions in the x direction.
		 * @param ny 			The number of subdivisions in the y direction.
		 * @param nz 			The number of subdivisions in the z direction.
		 */
		AABBGrid(const Eigen::AlignedBox3d &base_aabb, size_t nx, size_t ny, size_t nz);

		/**
		 * Get the AABB at the given grid coordinates.
		 * @param x 	The x coordinate.
		 * @param y 	The y coordinate.
		 * @param z 	The z coordinate.
		 * @return 		The AABB at the given grid coordinates, or std::nullopt if the coordinates are out of bounds.
		 */
		[[nodiscard]] std::optional<Eigen::AlignedBox3d> getAABB(const Eigen::Vector3i &coord) const;

		/**
		 * Given a 3D vector/point, get the grid coordinates of the AABB that contains it.
		 *
		 *	@param point 	The point to get the grid coordinates of.
		 *	@return 		The grid coordinates of the AABB that contains the given point, or std::nullopt if the point is out of bounds.
		 */
		[[nodiscard]] std::optional<Eigen::Vector3i> getGridCoordinates(const Eigen::Vector3d &point) const;

		/**
		 * Get the size of a single grid cell.
		 * @return The size of a single grid cell as a 3D vector.
		 */
		[[nodiscard]] Eigen::Vector3d cellSize() const;

		/**
		 * Given an AlignedBox3d, return an AlignedBox3i of all grid coordinates touched by the box.
		 *
		 * Min/max coordinates are inclusive.
		 */
		[[nodiscard]] std::optional<Eigen::AlignedBox3i> touchedCoordinates(const Eigen::AlignedBox3d &box) const;

	private:
		/**
		 * Get the minimum point of the AABB at the given grid coordinates.
		 * @param coord 	The grid coordinates.
		 * @return		The minimum point of the AABB at the given grid coordinates.
		 */
		[[nodiscard]] Eigen::Vector3d cellMin(const Eigen::Vector3i &coord) const;
	};

}

#endif //MGODPL_GRIDAABB_H
