// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_GRIDAABB_H
#define MGODPL_GRIDAABB_H

#include <optional>

#include "AABB.h"
#include "RangeInclusive.h"

namespace mgodpl::math {

	/**
	 * A struct that defines a regular grid of axis-aligned boxes,
	 * based off of a base AABB with some given natural number of
	 * subdivisions in each dimension.
	 */
	class AABBGrid {

		AABBd base_aabb;
		size_t nx, ny, nz;

	public:
		/**
		 * Create a new AABBGrid.
		 * @param base_aabb 	The base AABB.
		 * @param nx 			The number of subdivisions in the x direction.
		 * @param ny 			The number of subdivisions in the y direction.
		 * @param nz 			The number of subdivisions in the z direction.
		 */
		AABBGrid(const AABBd &base_aabb, size_t nx, size_t ny, size_t nz);

		/**
		 * Get the AABB at the given grid coordinates.
		 * @param x 	The x coordinate.
		 * @param y 	The y coordinate.
		 * @param z 	The z coordinate.
		 * @return 		The AABB at the given grid coordinates, or std::nullopt if the coordinates are out of bounds.
		 */
		[[nodiscard]] std::optional<AABBd> getAABB(const Vec3i &coord) const;

		/**
		 * Get the AABB that covers the given AABBi in the grid.
		 * @param box 	The AABBi to get the AABB for.
		 * @return 		The AABB that covers the given AABBi in the grid, or std::nullopt if the AABBi is out of bounds.
		 */
		[[nodiscard]] std::optional<AABBd> getAABB(const AABBi &box) const;

		/**
		 * Given a 3D vector/point, get the grid coordinates of the AABB that contains it.
		 *
		 *	@param point 	The point to get the grid coordinates of.
		 *	@return 		The grid coordinates of the AABB that contains the given point, or std::nullopt if the point is out of bounds.
		 */
		[[nodiscard]] std::optional<Vec3i> getGridCoordinates(const Vec3d &point) const;

		/**
		 * Get the size of a single grid cell.
		 * @return The size of a single grid cell as a 3D vector.
		 */
		[[nodiscard]] Vec3d cellSize() const;

		/**
		 * Get the size of the grid.
		 */
		[[nodiscard]] Vec3i size() const;

		/**
		 * Given an AlignedBox3d, return an AlignedBox3i of all grid coordinates touched by the box.
		 *
		 * Min/max coordinates are inclusive.
		 */
		[[nodiscard]] std::optional<AABBi> touchedCoordinates(const AABBd &box) const;

		const AABBd &baseAABB() const;



	private:
		/**
		 * Get the minimum point of the AABB at the given grid coordinates.
		 * @param coord 	The grid coordinates.
		 * @return		The minimum point of the AABB at the given grid coordinates.
		 */
		[[nodiscard]] Vec3d cellMin(const Vec3i &coord) const;
	};

}

#endif //MGODPL_GRIDAABB_H
