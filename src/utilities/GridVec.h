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
//#include <Eigen/Core>

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
	 * Get the value at the given grid coordinates.
	 * @param x The x coordinate.
	 * @param y The y coordinate.
	 * @param z The z coordinate.
	 * @return The value at the given grid coordinates.
	 */
	typename std::vector<T>::reference operator[](const std::array<size_t, 3> &coord) {
		assert(coord[0] < nx && coord[1] < ny && coord[2] < nz);

		size_t index = coord[0] + coord[1] * nx + coord[2] * nx * ny;

		return grid.at(index);
	}

	/**
	 * Get the value at the given grid coordinates. (const)
	 * @param x The x coordinate.
	 * @param y The y coordinate.
	 * @param z The z coordinate.
	 * @return The value at the given grid coordinates.
	 */
	typename std::vector<T>::const_reference operator[](const std::array<size_t, 3> &coord) const {
		assert(coord[0] < nx && coord[1] < ny && coord[2] < nz);

		size_t index = coord[0] + coord[1] * nx + coord[2] * nx * ny;

		return grid.at(index);
	}

	const std::array<size_t, 3> size() const {
		return {nx, ny, nz};
	}
};

#endif //MGODPL_GRIDVEC_H
