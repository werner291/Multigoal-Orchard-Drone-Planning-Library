// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include "AABBGrid.h"

mgodpl::math::AABBGrid::AABBGrid(const Eigen::AlignedBox3d &base_aabb, size_t nx, size_t ny, size_t nz) :
	base_aabb(base_aabb), nx(nx), ny(ny), nz(nz) {
}

std::optional<Eigen::AlignedBox3d> mgodpl::math::AABBGrid::getAABB(const Eigen::Vector3i &coord) const {
	if (coord.x() < 0 || coord.x() >= nx || coord.y() < 0 || coord.y() >= ny || coord.z() < 0 || coord.z() >= nz) {
		return std::nullopt;
	} else {
		Eigen::Vector3d cell_min = cellMin(coord);
		Eigen::Vector3d cell_max = cell_min + cellSize();
		return {Eigen::AlignedBox3d(cell_min, cell_max)};
	}
}

Eigen::Matrix<double, 3, 1> mgodpl::math::AABBGrid::cellMin(const Eigen::Vector3i &coord) const {

	auto min = base_aabb.min();
	auto cell_size = cellSize();

	return {min.x() + (double) coord.x() * cell_size.x(),
													   min.y() + (double) coord.y() * cell_size.y(),
													   min.z() + (double) coord.z() * cell_size.z()};
}

Eigen::Vector3d mgodpl::math::AABBGrid::cellSize() const {
	return base_aabb.sizes()
					.cwiseQuotient(Eigen::Vector3d((double) nx, (double) ny, (double) nz));
}

std::optional<Eigen::Vector3i> mgodpl::math::AABBGrid::getGridCoordinates(const Eigen::Vector3d &point) const {

	if (!base_aabb.contains(point)) {
		return std::nullopt;
	} else {

		// Use scaling + floor to get the grid coordinates.

		// First, scale the point so that the base AABB is a unit cube.
		Eigen::Vector3d scaled_point = (point - base_aabb.min()).cwiseQuotient(base_aabb.sizes());

		// Then, scale the point so that the grid is a unit cube.
		Eigen::Vector3d grid_step = Eigen::Vector3d((double) nx, (double) ny, (double) nz);
		Eigen::Vector3d scaled_grid_point = scaled_point.cwiseProduct(grid_step);

		// Finally, floor the point to get the grid coordinates.
		Eigen::Vector3i grid_point = scaled_grid_point.cast<int>();

		return {grid_point};

	}

}

std::optional<Eigen::AlignedBox3i> mgodpl::math::AABBGrid::touchedCoordinates(const Eigen::AlignedBox3d &box) const {

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

	return {Eigen::AlignedBox3i(Eigen::Vector3i(min_x, min_y, min_z), Eigen::Vector3i(max_x, max_y, max_z))};

}
