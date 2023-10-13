// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#ifndef MGODPL_GRIDVEC_H
#define MGODPL_GRIDVEC_H

#include <array>
#include <vector>
#include <cstddef>
#include <cassert>
#include <Eigen/Core>
#include "math/grid_utils.h"

/**
 * A 3D grid vector.
 * @tparam T The type of the grid values.
 */
template<typename T>
class Grid3D {
	std::vector<T> grid;
	size_t nx, ny, nz;

public:
	/**
	 * Create a new 3D grid.
	 * @param nx The number of grid cells in the x direction.
	 * @param ny The number of grid cells in the y direction.
	 * @param nz The number of grid cells in the z direction.
	 */
	Grid3D(size_t nx, size_t ny, size_t nz) : grid(nx*ny*nz), nx(nx), ny(ny), nz(nz) {
	}

	/**
	 * Create a new 3D grid, filled with the given value.
	 */
	Grid3D(size_t nx, size_t ny, size_t nz, const T &value) : grid(nx*ny*nz, value), nx(nx), ny(ny), nz(nz) {
	}

	/**
	 * Create a new 3D grid, filled with the given value.
	 */
	Grid3D(const Eigen::Vector3i &size, const T &value) : grid(size[0]*size[1]*size[2], value), nx(size[0]), ny(size[1]), nz(size[2]) {}

	/**
	 * Get the value at the given grid coordinates.
	 * @param coord The grid coordinates.
	 * @return The value at the given grid coordinates.
	 */
	typename std::vector<T>::reference operator[](const Eigen::Vector3i &coord) {
		assert(in_bounds(coord));

		size_t index = coord[0] + coord[1] * nx + coord[2] * nx * ny;

		return grid.at(index);
	}

	/**
	 * Get the value at the given grid coordinates. (const)
	 * @param coord The grid coordinates.
	 * @return The value at the given grid coordinates.
	 */
	typename std::vector<T>::const_reference operator[](const Eigen::Vector3i &coord) const {
		assert(in_bounds(coord));

		size_t index = coord[0] + coord[1] * nx + coord[2] * nx * ny;

		return grid.at(index);
	}

	/**
	 * Get the size of the grid, in number of blocks per dimension.
	 *
	 * @return	The size of the grid, in number of blocks per dimension.
	 */
	const Eigen::Vector3i size() const {
		return {(int)nx, (int)ny, (int)nz};
	}

	/**
	 * Check whether a given grid coordinate is within the grid bounds.
	 * @param pt	The grid coordinate.
	 * @return	True/false depending on whether the grid coordinate is within the grid bounds.
	 */
	bool in_bounds(const Eigen::Vector3i& pt) const {
		return pt[0] >= 0 && pt[0] < nx && pt[1] >= 0 && pt[1] < ny && pt[2] >= 0 && pt[2] < nz;
	}

	/**
	 * Given a coordinate and a grid, determine if the value at the coordinate is different from any of its neighbors.
	 *
	 * If a voxel is at the boundary of the grid, this function will always return true.
	 *
	 * For example, we may use this to get rid of invisible interior voxels when rendering as a set of cubes.
	 *
	 * @tparam T 			The type of the grid values.
	 * @param coord 		The coordinate.
	 * @param grid 			The grid.
	 * @return 				True/false depending on whether the voxel has a different neighbor, or is at the boundary.
	 */
	bool voxel_has_different_neighbor(const Eigen::Vector3i &coord) const {
		const auto& v_center = (*this)[coord];
		const auto& size = grid.size();

		for (const auto& neighbor : mgodpl::grid_utils::neighbors(coord)) {
			if (!in_bounds(neighbor)) {
				return true;
			}

			if ((*this)[neighbor] != v_center) {
				return true;
			}
		}

		return false;
	}
};

#endif //MGODPL_GRIDVEC_H
