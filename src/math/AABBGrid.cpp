// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include "AABBGrid.h"

namespace mgodpl::math {
	mgodpl::math::AABBGrid::AABBGrid(const AABBd &base_aabb, size_t nx, size_t ny, size_t nz)
			: base_aabb(base_aabb), nx(nx), ny(ny), nz(nz) {
	}

	std::optional<AABBd> mgodpl::math::AABBGrid::getAABB(const Vec3i &coord) const {
		if (coord.x() < 0 || coord.x() >= nx || coord.y() < 0 || coord.y() >= ny || coord.z() < 0 || coord.z() >= nz) {
			return std::nullopt;
		} else {
			Vec3d cell_min = cellMin(coord);
			Vec3d cell_max = cell_min + cellSize();
			return {AABBd(cell_min, cell_max)};
		}
	}

	Vec3d mgodpl::math::AABBGrid::cellMin(const Vec3i &coord) const {

		auto min = base_aabb.min();
		auto cell_size = cellSize();

		return min + Vec3d(coord.x(), coord.y(), coord.z()) * cell_size;
	}

	Vec3d mgodpl::math::AABBGrid::cellSize() const {
		return base_aabb.size() / Vec3d((double) nx, (double) ny, (double) nz);
	}

	std::optional<Vec3i> mgodpl::math::AABBGrid::getGridCoordinates(const Vec3d &point) const {

		if (!base_aabb.contains(point)) {
			return std::nullopt;
		} else {

			// Use scaling + floor to get the grid coordinates.

			// First, scale the point so that the base AABB is a unit cube.
			Vec3d scaled_point = (point - base_aabb.min()) / (base_aabb.size());

			// Then, scale the point so that the grid is a unit cube.
			Vec3d grid_step = Vec3d((double) nx, (double) ny, (double) nz);
			Vec3d scaled_grid_point = scaled_point * (grid_step);

			// Finally, floor the point to get the grid coordinates.
			Vec3i grid_point = scaled_grid_point.cast<int>();

			return {grid_point};

		}

	}

	std::optional<AABBi> mgodpl::math::AABBGrid::touchedCoordinates(const AABBd &box) const {

		// First, check if the AABBs even intersect.
		if (!base_aabb.intersects(box)) {
			return std::nullopt;
		}

		int min_x = std::max(0, (int) std::floor((box.min().x() - base_aabb.min().x()) / cellSize().x()));
		int min_y = std::max(0, (int) std::floor((box.min().y() - base_aabb.min().y()) / cellSize().y()));
		int min_z = std::max(0, (int) std::floor((box.min().z() - base_aabb.min().z()) / cellSize().z()));

		int max_x = std::min((int) nx - 1, (int) std::floor((box.max().x() - base_aabb.min().x()) / cellSize().x()));
		int max_y = std::min((int) ny - 1, (int) std::floor((box.max().y() - base_aabb.min().y()) / cellSize().y()));
		int max_z = std::min((int) nz - 1, (int) std::floor((box.max().z() - base_aabb.min().z()) / cellSize().z()));

		return {
				AABBi(Vec3i(min_x, min_y, min_z), Vec3i(max_x, max_y, max_z))
		};


	}

	std::optional<AABBd> AABBGrid::getAABB(const AABBi &box) const {
		auto lower = getAABB(box.min());
		auto upper = getAABB(box.max());

		if (lower.has_value() && upper.has_value()) {
			return AABBd(lower->min(), upper->max());
		} else {
			return std::nullopt;
		}
	}

	const AABBd &AABBGrid::baseAABB() const {
		return base_aabb;
	}

	Vec3i AABBGrid::size() const {
		return Vec3i(nx, ny, nz);
	}

	std::optional<int> AABBGrid::getCoordinateInDimension(const double &value, const int &dimension) const {


		if (value < base_aabb.min()[dimension] || value > base_aabb.max()[dimension]) {
			return std::nullopt;
		} else {
			return (value - base_aabb.min()[dimension]) / cellSize()[dimension];

		}


	}

	int AABBGrid::dim_coord_unckeched(const double &value, const int &dimension) const {
		return (int) std::floor((value - base_aabb.min()[dimension]) / cellSize()[dimension]);
	}

}