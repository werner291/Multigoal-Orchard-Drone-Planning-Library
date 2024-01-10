// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/9/24.
//

#include <gtest/gtest.h>
#include <random_numbers/random_numbers.h>

#include "../../src/planning/LatitudeLongitudeGrid.h"

using namespace mgodpl;
using namespace mgodpl::spherical_geometry;

LatitudeRange gen_latitude_range(random_numbers::RandomNumberGenerator &rng) {
	double lat_min = rng.uniformReal(-M_PI / 2.0, M_PI / 2.0);
	double lat_max = rng.uniformReal(-M_PI / 2.0, M_PI / 2.0);
	if (lat_min > lat_max) {
		std::swap(lat_min, lat_max);
	}
	return {lat_min, lat_max};
}

LongitudeRange gen_longitude_range(random_numbers::RandomNumberGenerator &rng) {
	double lon_min = rng.uniformReal(-M_PI, M_PI);
	double lon_max = rng.uniformReal(-M_PI, M_PI);
	return {lon_min, lon_max};
}

LatLonGrid gen_empty_grid(random_numbers::RandomNumberGenerator &rng) {
	return {
			gen_latitude_range(rng),
			gen_longitude_range(rng),
			static_cast<size_t>(rng.uniformInteger(5, 20)),
			static_cast<size_t>(rng.uniformInteger(5, 20))
	};
}

RelativeVertex gen_relative_vertex(random_numbers::RandomNumberGenerator &rng) {

	// Generate a unit vector uniformly using gaussian distributions.
	math::Vec3d v = math::Vec3d(
			rng.gaussian01(),
			rng.gaussian01(),
			rng.gaussian01()
	).normalized();

	return {
			longitude(v),
			latitude(v)
	};

}

RelativeVertex gen_relative_vertex_in_ranges(random_numbers::RandomNumberGenerator &rng,
											 LongitudeRange longitude_range,
											 LatitudeRange latitude_range) {
	return {
			.longitude = longitude_range.interpolate(rng.uniform01()).longitude,
			.latitude = latitude_range.interpolate(rng.uniform01())
	};
}

RelativeVertex random_point_in_triangle(random_numbers::RandomNumberGenerator &rng,
										const PaddedSphereTriangle &triangle) {

	// Generate a random point in the triangle.
	const Longitude lon = triangle.longitude_range().interpolate(rng.uniform01());
	assert(triangle.longitude_range().contains(lon));
	const auto lat = triangle.latitude_range_at_longitude(lon).interpolate(rng.uniform01());

	return {
			.longitude = lon.longitude,
			.latitude = lat
	};
}

TEST(LatitudeLongitudeGridTests, test_coordinates) {

	random_numbers::RandomNumberGenerator rng(42);

	for (int repeat = 0; repeat < 100; ++repeat) {

		auto grid = gen_empty_grid(rng);

		for (size_t lon_i = 0; lon_i < grid.longitude_cells; lon_i++) {
			auto range = grid.longitude_range_of_cell(lon_i);

			// Generate a few random longitudes in the range.
			for (int i = 0; i < 10; i++) {
				double t = rng.uniformReal(0, 1);
				ASSERT_TRUE(grid.to_grid_longitude(range.interpolate(t).longitude) == lon_i);
			}
		}

		for (size_t lat_i = 0; lat_i < grid.latitude_cells; lat_i++) {
			auto range = grid.latitude_range_of_cell(lat_i);

			// Generate a few random latitudes in the range.
			for (int i = 0; i < 10; i++) {
				double t = rng.uniformReal(0, 1);
				ASSERT_TRUE(grid.to_grid_latitude(range.interpolate(t)) == lat_i);
			}
		}
	}

}

TEST(LatitudeLongitudeGridTests, test_insertion) {

	random_numbers::RandomNumberGenerator rng(42);

	for (int repeat = 0; repeat < 100; ++repeat) {

		double long_start = rng.uniformReal(-M_PI, M_PI);
		double long_length = rng.uniformReal(0.0, M_PI);
		double lat_radius = rng.uniformReal(0.5, M_PI / 2.0);

		LatLonGrid grid{
				{-lat_radius, lat_radius},
				{long_start, wrap_angle(long_start + long_length)},
				static_cast<size_t>(rng.uniformInteger(5, 20)),
				static_cast<size_t>(rng.uniformInteger(5, 20))
		};

		// Generate a random spherical triangle.
		std::array<RelativeVertex, 3> vertices{
				gen_relative_vertex_in_ranges(rng, grid.longitude_range, grid.latitude_range),
				gen_relative_vertex_in_ranges(rng, grid.longitude_range, grid.latitude_range),
				gen_relative_vertex_in_ranges(rng, grid.longitude_range, grid.latitude_range)
		};

		// Sort by longitude; use signed_longitude_difference for stuff that crosses the 180 meridian.
		std::sort(vertices.begin(), vertices.end(), [](const auto &a, const auto &b) {
			return signed_longitude_difference(a.longitude, b.longitude) < 0;
		});

		PaddedSphereTriangle triangle{
				vertices,
				rng.uniform01() < 0.5 ? 0.0 : rng.uniformReal(0.0, 0.1)
		};

		// Insert it.
		grid.insert_triangle(triangle);

		// Generate a few random points in the triangle and check that the corresponding grid cell is occupied.
		for (int i = 0; i < 100; i++) {
			RelativeVertex point = random_point_in_triangle(rng, triangle);
			assert(triangle.longitude_range().contains(point.longitude));

			if (grid.longitude_range.contains(point.longitude) && grid.latitude_range.contains(point.latitude)) {
				auto grid_cell = grid.to_grid_index(point);
				ASSERT_EQ(grid.cells[grid.cell_index(grid_cell)].triangles.size(), 1);
			}
		}

		// Also check the longitude columns before and after:
		size_t lon_min = grid.to_grid_longitude(grid.longitude_range.clamp(triangle.longitude_range().start));
		size_t lon_max = grid.to_grid_longitude(grid.longitude_range.clamp(triangle.longitude_range().end));
		size_t lon_cell = lon_min;

		// Check that the cells surrounding the triangle are empty:
		while (true) {

			auto lats = triangle.latitude_range_over_longitude_range(grid.longitude_range_of_cell(lon_cell));

			size_t lat_min = grid.to_grid_latitude(grid.latitude_range.clamp(lats.min));
			size_t lat_max = grid.to_grid_latitude(grid.latitude_range.clamp(lats.max));

			// Look at the cells above and below the triangle.
			if (lat_min > 0) {
				ASSERT_EQ(grid.cells[grid.cell_index({.lat_i=static_cast<size_t>(lat_min -
																				 1), .lon_i=lon_cell})].triangles.size(),
						  0);
			}

			if (lat_max + 1 < grid.latitude_cells) {
				auto lat_range = grid.latitude_range_of_cell(lat_max + 1);
				auto lon_range = grid.longitude_range_of_cell(lon_cell);
				ASSERT_EQ(grid.cells[grid.cell_index({.lat_i=static_cast<size_t>(lat_max +
																				 1), .lon_i=lon_cell})].triangles.size(),
						  0);
			}

			if (lon_cell == lon_max) {
				break;
			} else {
				lon_cell = (lon_cell + 1) % grid.longitude_cells;
			}

		}

		for (size_t lat = 0; lat < grid.latitude_cells; lat++) {
			if (lon_min > 0) {
				ASSERT_EQ(grid.cells[grid.cell_index({.lat_i= static_cast<size_t>(lat), .lon_i= static_cast<size_t>(
						lon_min - 1)})].triangles.size(),
						  0);
			}

			if (lon_max + 1 < grid.longitude_cells) {
				ASSERT_EQ(grid.cells[grid.cell_index({.lat_i= static_cast<size_t>(lat),
															 .lon_i= static_cast<size_t>(lon_max +
																						 1)})].triangles.size(), 0);
			}
		}

		// For all grid cells, if the four neighboring cells are full, they should be marked as fully blocked.
		for (size_t lat = 1; lat + 1 < grid.latitude_cells; lat++) {
			for (size_t lon = 1; lon + 1 < grid.longitude_cells; lon++) {
				if (!grid.cells[grid.cell_index({.lat_i=lat - 1, .lon_i=lon})].triangles.empty() &&
					!grid.cells[grid.cell_index({.lat_i=lat + 1, .lon_i=lon})].triangles.empty() &&
					!grid.cells[grid.cell_index({.lat_i=lat, .lon_i=lon + 1})].triangles.empty() &&
					!grid.cells[grid.cell_index({.lat_i=lat, .lon_i=lon - 1})].triangles.empty()) {
					ASSERT_TRUE(grid.cells[grid.cell_index({.lat_i=lat, .lon_i=lon})].fully_blocked);
				}
			}
		}

	}

}
