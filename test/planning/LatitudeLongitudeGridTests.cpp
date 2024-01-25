// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/9/24.
//

#include <gtest/gtest.h>
#include <random_numbers/random_numbers.h>
#include <fcl/math/bv/OBB.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/collision.h>

#include "../../src/planning/LatitudeLongitudeGrid.h"
#include "../../src/experiment_utils/TreeMeshes.h"
#include "../../src/math/Triangle.h"
#include "../../src/experiment_utils/mesh_utils.h"

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
			rng.uniformReal(0.0, 0.1),
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
										const Triangle &triangle) {


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

TEST(LatitudeLongitudeGridTests, test_single_fullcells) {

	random_numbers::RandomNumberGenerator rng(42);

	for (int repeat = 0; repeat < 100; ++repeat) {

		double long_start = rng.uniformReal(-M_PI, M_PI);
		double long_length = rng.uniformReal(0.0, M_PI);
		double lat_radius = rng.uniformReal(0.5, M_PI / 2.0);

		LatLonGrid grid{
				{-lat_radius, lat_radius},
				{long_start, wrap_angle(long_start + long_length)},
				rng.uniformReal(0,0.1),
				static_cast<size_t>(rng.uniformInteger(5, 20)),
				static_cast<size_t>(rng.uniformInteger(5, 20))
		};

		// Generate a random triangle
		Triangle triangle {
				math::Vec3d {rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0)},
				math::Vec3d {rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0)},
				math::Vec3d {rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0)}
		};

		// Insert it.
		grid.insert_triangle(triangle);

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

fcl::Transform3d probe_insert_tf(const math::Vec3d& center, const RelativeVertex& vertex, double cylinder_height) {

	math::Vec3d dir = vertex.to_cartesian();

	fcl::Transform3d fcl_transform;
	fcl_transform.setIdentity();
	fcl_transform.translate(fcl::Vector3d(center.x(),center.y(),center.z()));
	fcl_transform.rotate(fcl::Quaterniond::FromTwoVectors(fcl::Vector3d(0, 0, 1), fcl::Vector3d(dir.x(), dir.y(), dir.z())));
	fcl_transform.translate(fcl::Vector3d(0.0, 0.0, cylinder_height / 2.0));

	return fcl_transform;
}

TEST(LatitudeLongitudeGridTests, test_single_bvhcomparison) {

	random_numbers::RandomNumberGenerator rng(42);

	for (int repeat = 0; repeat < 100; ++repeat) {

		double long_start = -M_PI/2.0; //rng.uniformReal(-M_PI, M_PI);
		double long_length = M_PI;//rng.uniformReal(0.0, M_PI);
		double lat_radius = M_PI / 4.0; //rng.uniformReal(0.5, M_PI / 2.0);

		double arm_radius = 0.01;//rng.uniformReal(0.01, 0.1);

		LatLonGrid grid{
				{-lat_radius, lat_radius},
				{long_start, wrap_angle(long_start + long_length)},
				arm_radius,
				20,//static_cast<size_t>(rng.uniformInteger(5, 20)),
				20//static_cast<size_t>(rng.uniformInteger(5, 20))
		};

		std::cout << "Grid lats: " << grid.latitude_range.min << " - " << grid.latitude_range.max << std::endl;
		std::cout << "Grid longs: " << grid.longitude_range.start << " - " << grid.longitude_range.end << std::endl;

		// Generate a random triangle
		Triangle triangle {
				math::Vec3d {rng.uniformReal(0.0, 5.0), rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0)},
				math::Vec3d {rng.uniformReal(0.0, 5.0), rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0)},
				math::Vec3d {rng.uniformReal(0.0, 5.0), rng.uniformReal(-5.0, 5.0), rng.uniformReal(-5.0, 5.0)}
		};

		std::cout << "Polygon({"
			<< "(" << triangle.vertices[0].x() << ", " << triangle.vertices[0].y() << ", " << triangle.vertices[0].z() << "), "
			<< "(" << triangle.vertices[1].x() << ", " << triangle.vertices[1].y() << ", " << triangle.vertices[1].z() << "), "
			<< "(" << triangle.vertices[2].x() << ", " << triangle.vertices[2].y() << ", " << triangle.vertices[2].z() << ")"
			<< "})" << std::endl;

		// Insert it.
		grid.insert_triangle(triangle);

		std::cout << "{";

		// First, print the lat-lon grid in a way that Geogebra understands.
		for (size_t lat = 0; lat < grid.latitude_cells; lat++) {
			for (size_t lon = 0; lon < grid.longitude_cells; lon++) {
				if (!grid.cells[grid.cell_index({.lat_i=lat, .lon_i=lon})].triangles.empty()) {
					auto lon_range = grid.longitude_range_of_cell(lon);
					auto lat_range = grid.latitude_range_of_cell(lat);

					math::Vec3d top_left = RelativeVertex {
						.longitude = lon_range.start,
						.latitude = lat_range.max
					}.to_cartesian();

					math::Vec3d top_right = RelativeVertex {
						.longitude = lon_range.end,
						.latitude = lat_range.max
					}.to_cartesian();

					math::Vec3d bottom_left = RelativeVertex {
						.longitude = lon_range.start,
						.latitude = lat_range.min
					}.to_cartesian();

					math::Vec3d bottom_right = RelativeVertex {
						.longitude = lon_range.end,
						.latitude = lat_range.min
					}.to_cartesian();

					std::cout << "Polyline({"
						<< "(" << top_left.x() << ", " << top_left.y() << ", " << top_left.z() << "), "
						<< "(" << top_right.x() << ", " << top_right.y() << ", " << top_right.z() << "), "
						<< "(" << bottom_right.x() << ", " << bottom_right.y() << ", " << bottom_right.z() << "), "
						<< "(" << bottom_left.x() << ", " << bottom_left.y() << ", " << bottom_left.z() << "),"
						<< "(" << top_left.x() << ", " << top_left.y() << ", " << top_left.z() << ")"
						<< "})";

					// If not last, emit a comma:
					if (lat != grid.latitude_cells - 1 || lon != grid.longitude_cells - 1) {
						std::cout << ", ";
					}

				}
			}
		}
		// End of list:
		std::cout << "}" << std::endl;

		fcl::Cylinderd fcl_cylinder(arm_radius, 10.0);

		// Build a BVHModel with that single triangle in it:
		fcl::BVHModel<fcl::OBBd> model;
		model.beginModel();
		model.addTriangle(
				fcl::Vector3d(triangle.vertices[0].x(), triangle.vertices[0].y(), triangle.vertices[0].z()),
				fcl::Vector3d(triangle.vertices[1].x(), triangle.vertices[1].y(), triangle.vertices[1].z()),
				fcl::Vector3d(triangle.vertices[2].x(), triangle.vertices[2].y(), triangle.vertices[2].z())
		);
		model.endModel();

		// Generate 100 latitude/longitude tests:
		for (int i = 0; i < 100; i++) {
			double lon = grid.longitude_range.interpolate(rng.uniform01()).longitude;
			double lat = grid.latitude_range.interpolate(rng.uniform01());

			auto probe_tf = probe_insert_tf(math::Vec3d(0, 0, 0), RelativeVertex{lon, lat}, fcl_cylinder.lz);

			// Do the collision check
			fcl::CollisionRequestd request;
			fcl::CollisionResultd result;
			fcl::collide(&fcl_cylinder, probe_tf, &model, fcl::Transform3d::Identity(), request, result);

			// Some debug prints to vizualize in Geogebra.

			fcl::Vector3d probe_base = probe_tf * fcl::Vector3d(0, 0, -fcl_cylinder.lz/2.0);
			fcl::Vector3d probe_tip = probe_tf * fcl::Vector3d(0, 0, fcl_cylinder.lz/2.0);

			std::cout << "Cylinder((" << probe_base.x() << ", " << probe_base.y() << ", " << probe_base.z() << "), (" << probe_tip.x() << ", " << probe_tip.y() << ", " << probe_tip.z() << "), " << fcl_cylinder.radius << ")" << std::endl;

			bool grid_cell_is_empty = grid.cells[grid.cell_index({.lat_i=grid.to_grid_latitude(lat), .lon_i=grid.to_grid_longitude(lon)})].triangles.empty();

			// If the cell is empty, there must be no collision.
			if (grid_cell_is_empty) {
				ASSERT_FALSE(result.isCollision());
			}

			// If there is a collision, the cell must be non-empty.
			if (result.isCollision()) {
				ASSERT_FALSE(grid_cell_is_empty);
			}

		}
	}
}

TEST(LatitudeLongitudeGridTests, OBBdComparisonTest) {

	random_numbers::RandomNumberGenerator rng(42);

	// Grab a tree from the test data.
	const auto& model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	// Extract the branches.
	std::vector<mgodpl::Triangle> triangles;

	for (const auto& triangle : model.trunk_mesh.triangles) {

		const auto& v1 = model.trunk_mesh.vertices[triangle.vertex_indices[0]];
		const auto& v2 = model.trunk_mesh.vertices[triangle.vertex_indices[1]];
		const auto& v3 = model.trunk_mesh.vertices[triangle.vertex_indices[2]];

		triangles.push_back(mgodpl::Triangle{
			.vertices = {
					math::Vec3d {v1.x, v1.y, v1.z},
					math::Vec3d {v2.x, v2.y, v2.z},
					math::Vec3d {v3.x, v3.y, v3.z}
			}
		});

	}

	const math::Vec3d& center = mesh_aabb(model.leaves_mesh).center();

	fcl::BVHModel<fcl::OBBd> model_obb;
	model_obb.beginModel();
	for (const auto& triangle : triangles) {
		model_obb.addTriangle(
				fcl::Vector3d(triangle.vertices[0].x(), triangle.vertices[0].y(), triangle.vertices[0].z()),
				fcl::Vector3d(triangle.vertices[1].x(), triangle.vertices[1].y(), triangle.vertices[1].z()),
				fcl::Vector3d(triangle.vertices[2].x(), triangle.vertices[2].y(), triangle.vertices[2].z())
		);
	}
	model_obb.endModel();


	for (const math::Vec3d& fruit: computeFruitPositions(model)) {
	bool found_one = false;

		double longitude = spherical_geometry::longitude(fruit, center);

		// Create the LatitudeLongituteGrid

		const double arm_radius = 0.025;

		auto start_time = std::chrono::high_resolution_clock::now();
		LatLonGrid grid = LatLonGrid::from_triangles(triangles, fruit, arm_radius, 20, 20);
		auto end_time = std::chrono::high_resolution_clock::now();
		std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms" << std::endl;

		// Find the empty cells:
		for (size_t lat = 0; lat < grid.latitude_cells; lat++) {
			for (size_t lon = 0; lon < grid.longitude_cells; lon++) {
				if (grid.cells[grid.cell_index({.lat_i=lat, .lon_i=lon})].triangles.empty()) {
					// Check that the cell is empty in the OBBd model.

					math::Vec3d dir = RelativeVertex {
						.longitude = grid.longitude_range_of_cell(lon).interpolate(0.5).longitude,
						.latitude = grid.latitude_range_of_cell(lat).interpolate(0.5)
					}.to_cartesian();

					fcl::Cylinderd fcl_cylinder(arm_radius, 10.0);

					fcl::Transform3d fcl_transform;
					fcl_transform.setIdentity();
					fcl_transform.translate(fcl::Vector3d(
							center.x(),
							center.y(),
							center.z()
					));
					fcl_transform.rotate(fcl::Quaterniond::FromTwoVectors(fcl::Vector3d(0, 0, 1), fcl::Vector3d(dir.x(), dir.y(), dir.z())));

					// Perform the collision check.
					fcl::CollisionRequestd request;
					fcl::CollisionResultd result;
					fcl::collide(&fcl_cylinder, fcl_transform, &model_obb, fcl::Transform3d::Identity(), request, result);

					ASSERT_FALSE(result.isCollision());

					found_one = true;
				}
			}
		}

		if (found_one) {
			std::cout << "Found empty cells for fruit at " << fruit << std::endl;
		} else {
			std::cout << "No empty cells found for fruit at " << fruit << std::endl;
		}
	}

}
