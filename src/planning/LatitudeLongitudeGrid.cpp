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

	using namespace spherical_geometry;

	math::Vec3d slerp(const math::Vec3d &a, const math::Vec3d &b, double t) {
		double theta = acos(a.dot(b));
		return (a * sin((1 - t) * theta) + b * sin(t * theta)) / sin(theta);
	}

	double latitude_at_longitude(const Edge &edge, double longitude) {
		// Just like intersection_longitude, we can reduce this problem to linear algebra.

		// First, we generate a normal for the plane that cuts through the given longitude.
		math::Vec3d lon_direction = math::Vec3d(
				cos(longitude + M_PI / 2.0),
				sin(longitude + M_PI / 2.0),
				0
		);

		// Now, get the intersection point of the edge with the plane.
		math::Vec3d intersection = edge.vertices[0].cross(edge.vertices[1]).cross(lon_direction);

		return spherical_geometry::latitude(intersection);
	}

	LatLonGrid LatLonGrid::from_triangles(const std::vector<Triangle> &triangles,
										  const math::Vec3d &center,
										  double arm_radius,
										  size_t latitude_steps,
										  size_t longitude_steps) {

		LatLonGrid grid{
				spherical_geometry::LatitudeRange(-M_PI / 2.0, M_PI / 2.0),
				spherical_geometry::LongitudeRange(-M_PI, M_PI),
				arm_radius,
				latitude_steps,
				longitude_steps
		};

		for (const auto &triangle: triangles) {
			grid.insert_triangle(mgodpl::Triangle {
					{
						triangle.vertices[0] - center,
						triangle.vertices[1] - center,
						triangle.vertices[2] - center
					}
			});
		}

		return grid;

	}

	void LatLonGrid::insert_triangle(const Triangle &triangle) {

		// Use scanline rasterization to find all grid cells affected by the triangle.
		// We will go longitude-row-by-longitude-row, and for each longitude row, we will
		// go latitude-by-latitude.

		// First, we establish a few basic properties about our triangle.
		std::array<double, 3> latitudes {
				spherical_geometry::latitude(triangle.vertices[0]),
				spherical_geometry::latitude(triangle.vertices[1]),
				spherical_geometry::latitude(triangle.vertices[2])
		};

		std::array<double, 3> longitudes {
				spherical_geometry::longitude(triangle.vertices[0]),
				spherical_geometry::longitude(triangle.vertices[1]),
				spherical_geometry::longitude(triangle.vertices[2])
		};

		std::array<double, 3> radii {
			triangle.vertices[0].norm(),
			triangle.vertices[1].norm(),
			triangle.vertices[2].norm()
		};

		std::array<double, 3> paddings {
				spherical_geometry::angular_padding(arm_radius, radii[0]),
				spherical_geometry::angular_padding(arm_radius, radii[1]),
				spherical_geometry::angular_padding(arm_radius, radii[2])
		};

		// Then, index by the signed longitude.
		std::array<size_t, 3> lon_ordering { 0,1,2 };
		std::sort(lon_ordering.begin(), lon_ordering.end(), [&](size_t a, size_t b) {
			return spherical_geometry::signed_longitude_difference(longitudes[a], longitudes[b]) < 0;
		});

		// Find the longitude range of the triangle.
		double leftmost_longitude = wrap_angle(longitudes[lon_ordering[0]] - paddings[lon_ordering[0]]);
		double rightmost_longitude = wrap_angle(longitudes[lon_ordering[2]] + paddings[lon_ordering[2]]);

		Edge short_edge1 = {
				triangle.vertices[lon_ordering[0]],
				triangle.vertices[lon_ordering[1]]
		};

		Edge short_edge2 = {
				triangle.vertices[lon_ordering[1]],
				triangle.vertices[lon_ordering[2]]
		};

		Edge long_edge = {
				triangle.vertices[lon_ordering[0]],
				triangle.vertices[lon_ordering[2]]
		};

		math::Vec3d short_edge1_normal = short_edge1.vertices[0].cross(short_edge1.vertices[1]);
		math::Vec3d short_edge2_normal = short_edge2.vertices[0].cross(short_edge2.vertices[1]);
		math::Vec3d long_edge_normal = long_edge.vertices[1].cross(long_edge.vertices[0]);

		// Apply the normals.
		short_edge1.vertices[0] = short_edge1.vertices[0] + short_edge1_normal * paddings[lon_ordering[0]];
		short_edge1.vertices[1] = short_edge1.vertices[1] + short_edge1_normal * paddings[lon_ordering[1]];

		short_edge2.vertices[0] = short_edge2.vertices[0] + short_edge2_normal * paddings[lon_ordering[1]];
		short_edge2.vertices[1] = short_edge2.vertices[1] + short_edge2_normal * paddings[lon_ordering[2]];

		long_edge.vertices[0] = long_edge.vertices[0] + long_edge_normal * paddings[lon_ordering[0]];
		long_edge.vertices[1] = long_edge.vertices[1] + long_edge_normal * paddings[lon_ordering[2]];

		spherical_geometry::LongitudeRange short_edge1_long_range(
				spherical_geometry::longitude(short_edge1.vertices[0]),
				spherical_geometry::longitude(short_edge1.vertices[1])
				);

		spherical_geometry::LongitudeRange short_edge2_long_range(
				spherical_geometry::longitude(short_edge2.vertices[0]),
				spherical_geometry::longitude(short_edge2.vertices[1])
				);

		spherical_geometry::LongitudeRange long_edge_long_range(
				spherical_geometry::longitude(long_edge.vertices[0]),
				spherical_geometry::longitude(long_edge.vertices[1])
				);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ArgumentSelectionDefects"
		std::array<spherical_geometry::LongitudeRange, 3> edge_padding_ranges {
				spherical_geometry::LongitudeRange {leftmost_longitude, short_edge1_long_range.start},
				spherical_geometry::LongitudeRange {short_edge1_long_range.end, short_edge2_long_range.start},

				spherical_geometry::LongitudeRange {short_edge2_long_range.end, rightmost_longitude}
		};
#pragma clang diagnostic pop

		// Iterate over all cells touched by the padded triangle.
		size_t longitude_cell = to_grid_longitude(leftmost_longitude);
		size_t lon_cell_max = to_grid_longitude(rightmost_longitude);

		while(true) {

			double lat_min = INFINITY;
			double lat_max = -INFINITY;

			spherical_geometry::LongitudeRange longitude_range_of_cell(
					meridian(longitude_cell).longitude,
					meridian(longitude_cell + 1).longitude
			);

			// Now, for each of the ranges, we'll expand the latitude range
			// of the cell to include that portion of the padding.

			for (size_t i = 0; i < 3; i++) {
				if (longitude_range_of_cell.overlaps(edge_padding_ranges[i])) {
					// The edge is in this cell.
					lat_min = std::min(lat_min, latitudes[lon_ordering[i]] - paddings[lon_ordering[i]]);
					lat_max = std::max(lat_max, latitudes[lon_ordering[i]] + paddings[lon_ordering[i]]);
				}
			}

			for (const double meridian_longitude : {longitude_range_of_cell.start, longitude_range_of_cell.end}) {

				if (short_edge1_long_range.contains(meridian_longitude)) {
					double lat = latitude_at_longitude(short_edge1, meridian_longitude);
					lat_min = std::min(lat_min, lat - paddings[lon_ordering[0]]);
					lat_max = std::max(lat_max, lat + paddings[lon_ordering[1]]);
				}

				if (short_edge2_long_range.contains(meridian_longitude)) {
					double lat = latitude_at_longitude(short_edge2, meridian_longitude);
					lat_min = std::min(lat_min, lat - paddings[lon_ordering[1]]);
					lat_max = std::max(lat_max, lat + paddings[lon_ordering[2]]);
				}

				if (long_edge_long_range.contains(meridian_longitude)) {
					double lat = latitude_at_longitude(long_edge, meridian_longitude);
					lat_min = std::min(lat_min, lat - paddings[lon_ordering[0]]);
					lat_max = std::max(lat_max, lat + paddings[lon_ordering[2]]);
				}

			}

			size_t lat_cell_min = to_grid_latitude(std::clamp(lat_min, latitude_range.min, latitude_range.max));
			size_t lat_cell_max = to_grid_latitude(std::clamp(lat_max, latitude_range.min, latitude_range.max));

			if (lat_cell_min == lat_cell_max) {
				// Single cell
				cells[lat_cell_min * longitude_cells + longitude_cell].triangles.push_back(triangle);
			} else {
				// Add to min and max.
				cells[lat_cell_min * longitude_cells + longitude_cell].triangles.push_back(triangle);
				cells[lat_cell_max * longitude_cells + longitude_cell].triangles.push_back(triangle);

				// If there are between steps, mark them fully-occupied.
				for (size_t lat_cell = lat_cell_min + 1; lat_cell < lat_cell_max; lat_cell++) {
					cells[lat_cell * longitude_cells + longitude_cell].triangles.push_back(triangle);
					if (longitude_cell != lon_cell_max) {
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

	spherical_geometry::LatitudeRange LatLonGrid::latitude_range_of_cell(size_t i) {
		return {
				latitude_range.interpolate(static_cast<double>(i) / (double) latitude_cells),
				latitude_range.interpolate(static_cast<double>(i + 1) / (double) latitude_cells)
		};
	}
}
