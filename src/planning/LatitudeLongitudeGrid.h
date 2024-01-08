// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/7/24.
//

#ifndef MGODPL_LATITUDELONGITUDEGRID_H
#define MGODPL_LATITUDELONGITUDEGRID_H

#include "quickprobe.h"
#include "geometry.h"
#include "spherical_geometry.h"

namespace mgodpl {

	/**
	 * Represents a grid of cells in latitude and longitude.
	 */
	struct LatLonGrid {

		spherical_geometry::LatitudeRange latitude_range;
		spherical_geometry::LongitudeRange longitude_range;
		size_t latitude_cells;
		size_t longitude_cells;

		LatLonGrid(spherical_geometry::LatitudeRange latitude_range,
				   spherical_geometry::LongitudeRange longitude_range,
				   size_t latitude_steps,
				   size_t longitude_steps)
				: latitude_range(latitude_range),
				  longitude_range(longitude_range),
				  latitude_cells(latitude_steps),
				  longitude_cells(longitude_steps) {
			cells.resize(latitude_steps * longitude_steps);
		}

		[[nodiscard]] size_t to_grid_latitude(const double latitude) const {
			assert(latitude_range.contains(latitude));
			double lat_rescaled = (latitude - latitude_range.min) / (latitude_range.max - latitude_range.min);

			auto cell_idx = std::floor(lat_rescaled * (double) latitude_cells);

			// Clamp it into the valid range.
			if (cell_idx < 0) {
				cell_idx = 0;
			} else if (cell_idx >= latitude_cells) {
				cell_idx = latitude_cells - 1;
			}

			return static_cast<size_t>(cell_idx);
		}

		[[nodiscard]] size_t to_grid_longitude(const double longitude) const {
			assert(longitude_range.contains(longitude));
			double lon_rescaled = (longitude - longitude_range.start) / (longitude_range.end - longitude_range.start);

			auto cell_idx = std::floor(lon_rescaled * (double) longitude_cells);

			assert(cell_idx >= 0 && cell_idx < longitude_cells);

			return static_cast<size_t>(cell_idx);
		}

		[[nodiscard]] std::array<int, 2> to_gridcell(const spherical_geometry::RelativeVertex &vertex) const {
			return {
					to_grid_longitude(vertex.longitude),
					to_grid_latitude(vertex.latitude)
			};
		}

		struct Cell {
			std::vector<spherical_geometry::PaddedSphereTriangle> triangles;
			bool fully_blocked = false;
		};

		std::vector<Cell> cells;

		[[nodiscard]] inline size_t cell_index(size_t cell_x, size_t cell_y) const {
			assert(cell_x >= 0 && cell_x < longitude_cells);
			assert(cell_y >= 0 && cell_y < latitude_cells);
			return cell_y * longitude_cells + cell_x;
		}

		static LatLonGrid from_triangles(const std::vector<Triangle> &triangles,
										 const mgodpl::math::Vec3d &center,
										 double arm_radius,
										 size_t latitude_steps,
										 size_t longitude_steps);

		void insert_triangle(const spherical_geometry::PaddedSphereTriangle &triangle);

		void insert_triangle_expensive(const spherical_geometry::PaddedSphereTriangle &triangle);

		[[nodiscard]] inline spherical_geometry::Longitude meridian(size_t x) const {
			assert(x >= 0 && x <= longitude_cells);

			double t = (double) x / (double) longitude_cells;
			return longitude_range.interpolate(t);
		}

		[[nodiscard]] size_t count_all() const {
			size_t count = 0;
			for (const auto &cell: cells) {
				count += cell.triangles.size();
			}
			return count;
		}

		[[nodiscard]] size_t count_empty() const {
			size_t count = 0;
			for (const auto &cell: cells) {
				count += cell.triangles.empty();
			}
			return count;
		}

		[[nodiscard]] size_t count_fully_blocked() const {
			size_t count = 0;
			for (const auto &cell: cells) {
				count += cell.fully_blocked;
			}
			return count;
		}


	};
}

#endif //MGODPL_LATITUDELONGITUDEGRID_H
