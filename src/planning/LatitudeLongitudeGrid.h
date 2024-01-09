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

		/// A latitude/longitude grid cell index.
		struct GridIndex {
			size_t lat_i, lon_i;
		};

		/// A single cell in the grid.
		struct Cell {
			/// A vector of all triangle whose padded area intersects the vector.
			std::vector<spherical_geometry::PaddedSphereTriangle> triangles;
			/// Whether the cell is fully blocked by any triangle.
			bool fully_blocked = false;
		};

		/// The latitude range over which to take the grid.
		spherical_geometry::LatitudeRange latitude_range;
		/// The longitude range over which to take the grid.
		spherical_geometry::LongitudeRange longitude_range;
		/// The number of cells in the grid on the latitude axis.
		size_t latitude_cells;
		/// The number of cells in the grid on the longitude axis.
		size_t longitude_cells;
		/// The cells in the grid.
		std::vector<Cell> cells;

		/**
		 * Construct an empty latitude/longitude grid.
		 *
		 * @param latitude_range 		The latitude range over which to take the grid.
		 * @param longitude_range 		The longitude range over which to take the grid.
		 * @param latitude_steps 		The number of cells in the grid on the latitude axis.
		 * @param longitude_steps 		The number of cells in the grid on the longitude axis.
		 */
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

		/**
		 * Convert a latitude angle (in radians) to the latitude component of a grid cell index.
		 *
		 * The latitude must lie within the latitude range of the grid.
		 *
		 * @param latitude 		The latitude angle (in radians).
		 * @return 				The latitude component of a grid cell index.
		 */
		[[nodiscard]] size_t to_grid_latitude(const double latitude) const {
			assert(latitude_range.contains(latitude));

			// Map the latitude from `latitude_range` to [0, latitude_cells].
			double lat_rescaled = (latitude - latitude_range.min) / (latitude_range.max - latitude_range.min);

			// Floor it to get the cell index.
			auto cell_idx = std::floor(lat_rescaled * (double) latitude_cells);

			// Clamp it into the valid range (Cells on the boundary of the last row are considered to be in that row).
			if (cell_idx < 0) {
				cell_idx = 0;
			} else if (cell_idx >= latitude_cells) {
				cell_idx = latitude_cells - 1;
			}

			// Convert to size_t and return.
			return static_cast<size_t>(cell_idx);
		}

		/**
		 * Convert a longitude angle (in radians) to the longitude component of a grid cell index.
		 * @param longitude 		The longitude angle (in radians).
		 *
		 * The longitude must lie within the longitude range of the grid.
		 *
		 * @return The longitude component of a grid cell index.
		 */
		[[nodiscard]] size_t to_grid_longitude(const double longitude) const {
			assert(longitude_range.contains(longitude));

			// Map the longitude from `longitude_range` to [0, longitude_cells].
			double lon_rescaled = longitude_range.reverse_interpolate(longitude);

			// Floor it to get the cell index.
			auto cell_idx = std::floor(lon_rescaled * (double) longitude_cells);

			// Check that it's in the valid range. (TODO: When might it not be?)
			assert(cell_idx >= 0 && cell_idx < longitude_cells);

			// Convert to size_t and return.
			return static_cast<size_t>(cell_idx);
		}

		[[nodiscard]] GridIndex to_grid_index(const spherical_geometry::RelativeVertex &vertex) const {
			return {
					to_grid_latitude(vertex.latitude),
					to_grid_longitude(vertex.longitude)
			};
		}

		/**
		 * Convert a GridIndex to an index into the `cells` vector.
		 */
		[[nodiscard]] inline size_t cell_index(GridIndex gi) const {
			assert(gi.lon_i >= 0 && gi.lon_i < longitude_cells);
			assert(gi.lat_i >= 0 && gi.lat_i < latitude_cells);
			return gi.lat_i * longitude_cells + gi.lon_i;
		}

		/**
		 * Create a LatLonGrid from a vector of triangles, splitting said triangles up among the cells of the grid.
		 * @param triangles 			The triangles to insert into the grid.
		 * @param center 				The center of the sphere that we're projecting onto.
		 * @param arm_radius 			The radius of the arm that we're trying to insert into the tree.
		 * @param latitude_steps 		The number of cells in the grid on the latitude axis.
		 * @param longitude_steps 		The number of cells in the grid on the longitude axis.
		 * @return 						The resulting grid.
		 */
		static LatLonGrid from_triangles(const std::vector<Triangle> &triangles,
										 const mgodpl::math::Vec3d &center,
										 double arm_radius,
										 size_t latitude_steps,
										 size_t longitude_steps);

		/// Insert a single triangle into the grid.
		void insert_triangle(const spherical_geometry::PaddedSphereTriangle &triangle);

		/// Insert a single triangle, the hard way (if it is assumed to span multiple rows and columns of cells).
		void insert_triangle_expensive(const spherical_geometry::PaddedSphereTriangle &triangle);

		/// Compute the longitude of the left meridian/edge of a cell.
		[[nodiscard]] inline spherical_geometry::Longitude meridian(size_t x) const {
			assert(x >= 0 && x <= longitude_cells);

			double t = (double) x / (double) longitude_cells;
			return longitude_range.interpolate(t);
		}

		[[nodiscard]] inline spherical_geometry::LongitudeRange longitude_range_of_cell(size_t x) const {
			return {meridian(x).longitude, meridian(x+1).longitude};
		}

		/// Count the total number of triangles inserted across all cells.
		/// Note that triangles may be inserted into multiple cells, adding to their count.
		[[nodiscard]] size_t count_all() const {
			size_t count = 0;
			for (const auto &cell: cells) {
				count += cell.triangles.size();
			}
			return count;
		}

		/// Count the number of cells that are empty.
		[[nodiscard]] size_t count_empty() const {
			size_t count = 0;
			for (const auto &cell: cells) {
				count += cell.triangles.empty();
			}
			return count;
		}

		/// Count the number of cells that are fully blocked.
		[[nodiscard]] size_t count_fully_blocked() const {
			size_t count = 0;
			for (const auto &cell: cells) {
				count += cell.fully_blocked;
			}
			return count;
		}

		spherical_geometry::LatitudeRange latitude_range_of_cell(size_t i);
	};
}

#endif //MGODPL_LATITUDELONGITUDEGRID_H
