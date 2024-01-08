// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/7/24.
//
#include "LatitudeLongitudeGrid.h"
#include "spherical_geometry.h"
#include <algorithm>

namespace mgodpl {

	LatLonGrid LatLonGrid::from_triangles(const std::vector<Triangle> &triangles,
										  const math::Vec3d &center,
										  double arm_radius,
										  size_t latitude_steps,
										  size_t longitude_steps) {

		LatLonGrid grid{
				spherical_geometry::LatitudeRange(-M_PI / 2.0, M_PI / 2.0),
				spherical_geometry::LongitudeRange(-M_PI, M_PI),
				latitude_steps,
				longitude_steps
		};

		for (const auto &triangle: triangles) {
			auto padded_triangle = spherical_geometry::PaddedSphereTriangle::from_triangle(triangle,
																						   center,
																						   arm_radius);

			grid.insert_triangle(padded_triangle);
		}

		return grid;

	}

	void LatLonGrid::insert_triangle(const spherical_geometry::PaddedSphereTriangle &triangle) {

		auto ga = to_gridcell(triangle.vertices[0]);
		auto gb = to_gridcell(triangle.vertices[1]);
		auto gc = to_gridcell(triangle.vertices[2]);

		int min_lat = std::min(std::min(ga[1], gb[1]), gc[1]);
		int max_lat = std::max(std::max(ga[1], gb[1]), gc[1]);

		int min_lon = std::min(std::min(ga[0], gb[0]), gc[0]);
		int max_lon = std::max(std::max(ga[0], gb[0]), gc[0]);

		if (min_lat == max_lat && min_lon == max_lon) {
			// Single cell
			cells[min_lat * longitude_cells + min_lon].triangles.push_back(triangle);
		} else {
			if (min_lat == max_lat) {
				// Horizontal line
				for (int lon = min_lon; lon <= max_lon; lon++) {
					cells[min_lat * longitude_cells + lon].triangles.push_back(triangle);
				}
			} else if (min_lon == max_lon) {
				// Vertical line
				for (int lat = min_lat; lat <= max_lat; lat++) {
					cells[lat * longitude_cells + min_lon].triangles.push_back(triangle);
				}
			} else {
				this->insert_triangle_expensive(triangle);
			}
		}

	}

	void LatLonGrid::insert_triangle_expensive(const spherical_geometry::PaddedSphereTriangle &triangle) {
		// Use scanline rasterization to find all grid cells affected by the triangle.
		// We will go longitude-row-by-longitude-row, and for each longitude row, we will
		// go latitude-by-latitude.

		// Since triangles have ordered vertices in longitude,
		double lon_min = spherical_geometry::wrap_angle(triangle.vertices[0].longitude - triangle.angular_padding);
		double lon_max = spherical_geometry::wrap_angle(triangle.vertices[2].longitude + triangle.angular_padding);

		int lon_cell_min = to_grid_longitude(lon_min);
		int lon_cell_max = to_grid_longitude(lon_max);

		int middle_point_cell = to_grid_longitude(triangle.vertices[1].longitude);
		bool middle_passed = false;

		spherical_geometry::OrderedArcEdge edge_short1(triangle.vertices[0], triangle.vertices[1]);
		spherical_geometry::OrderedArcEdge edge_short2(triangle.vertices[1], triangle.vertices[2]);
		spherical_geometry::OrderedArcEdge edge_long(triangle.vertices[0], triangle.vertices[2]);

		// Adjust the edges to add padding. (TODO: This is a bit rough.)
		edge_short1.start.longitude = spherical_geometry::wrap_angle(edge_short1.start.longitude - triangle.angular_padding);
		edge_long.start.longitude = spherical_geometry::wrap_angle(edge_long.start.longitude - triangle.angular_padding);
		edge_short2.end.longitude = spherical_geometry::wrap_angle(edge_short2.end.longitude + triangle.angular_padding);
		edge_long.end.longitude = spherical_geometry::wrap_angle(edge_long.end.longitude + triangle.angular_padding);

		// Iterate over all cells touched by the padded triangle.
		size_t longitude_cell = lon_cell_min;

		while(true) {

			spherical_geometry::LongitudeRange longitude_range_of_cell(
					meridian(longitude_cell).longitude,
					meridian(longitude_cell + 1).longitude
			);

			auto long_restricted = edge_long.restrict(longitude_range_of_cell);

			spherical_geometry::LatitudeRange latitude_range_of_cell = long_restricted.latitude_range();

			if (longitude_range_of_cell.contains(triangle.vertices[1].longitude)) {
				latitude_range_of_cell.min = std::min(latitude_range_of_cell.min, triangle.vertices[1].latitude);
				latitude_range_of_cell.max = std::max(latitude_range_of_cell.max, triangle.vertices[1].latitude);
			} else {
				if (spherical_geometry::signed_longitude_difference(triangle.vertices[1].longitude,
																	longitude_range_of_cell.start) < 0) {
					latitude_range_of_cell.min = std::min(latitude_range_of_cell.min, triangle.vertices[1].latitude);
				} else {
					latitude_range_of_cell.max = std::max(latitude_range_of_cell.max, triangle.vertices[1].latitude);
				}
			}

			size_t lat_cell_min = to_grid_latitude(std::clamp(latitude_range_of_cell.min - triangle.angular_padding,
															 -M_PI / 2.0, M_PI / 2.0));
			size_t lat_cell_max = to_grid_latitude(std::clamp(latitude_range_of_cell.max + triangle.angular_padding,
															 -M_PI / 2.0, M_PI / 2.0));

			if (lat_cell_min == lat_cell_max) {
				// Single cell
				cells[lat_cell_min * longitude_cells + longitude_cell].triangles.push_back(triangle);
			} else {
				// Add to min and max.
				cells[lat_cell_min * longitude_cells + longitude_cell].triangles.push_back(triangle);
				cells[lat_cell_max * longitude_cells + longitude_cell].triangles.push_back(triangle);

				// If there are between steps, mark them fully-occupied.
				for (int lat_cell = lat_cell_min + 1; lat_cell < lat_cell_max; lat_cell++) {
					cells[lat_cell * longitude_cells + longitude_cell].triangles.push_back(triangle);
					if (longitude_cell != lon_cell_min && longitude_cell != lon_cell_max) {
						cells[lat_cell * longitude_cells + longitude_cell].fully_blocked = true;
					}
				}
			}

			if (longitude_cell == lon_cell_max) {
				break;
			} else {
				longitude_cell = (longitude_cell + 1) % longitude_cells;
			}
		}
	}
}
