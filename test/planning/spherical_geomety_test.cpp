// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12/1/23.
//

#include <gtest/gtest.h>
#include <random_numbers/random_numbers.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

#include "../../src/planning/longitude_sweep.h"
#include "../../src/experiment_utils/TreeMeshes.h"
#include "../../src/math/Quaternion.h"

using namespace mgodpl;
using namespace spherical_geometry;

/**
 * Generates a random arc edge that contains a given point.
 *
 * This function creates a random edge in 3D space that passes through a specified point.
 * The direction of the edge is determined randomly.
 *
 * @param a     The point in on the unit sphere that the edge should pass through.
 * @param rng   A reference to a random number generator to be used for creating the random edge.
 * @return      An Edge object representing the random edge that contains the point 'a'.
 */
Edge randomEdgeContainingPoint(math::Vec3d a, random_numbers::RandomNumberGenerator& rng)
{

    math::Vec3d rot_axis = a.cross(math::Vec3d(rng.gaussian01(), rng.gaussian01(), rng.gaussian01())).normalized();

    double angle1 = rng.uniformReal(-M_PI / 2.0, 0.0);
    double angle2 = rng.uniformReal(0.0, M_PI / 2.0);

    return Edge {
        {
            math::Quaterniond::fromAxisAngle(rot_axis, angle1).rotate(a),
            math::Quaterniond::fromAxisAngle(rot_axis, angle2).rotate(a)
        }
    };

}

//
//TEST(longitude_sweep_tests, intersection_longitude_test)
//{
//
//	random_numbers::RandomNumberGenerator rng(42);
//
//	// For 1000 iterations...
//	for (int i = 0; i < 1000; ++i)
//	{
//
//		// Construct a random unit vector:
//		math::Vec3d a(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
//		a = a.normalized();
//
//		Edge A1 = randomEdgeContainingPoint(a, rng);
//		Edge B1 = randomEdgeContainingPoint(a, rng);
//
//		math::Vec3d intersection = Edge_intersection(A1, B1);
//
//		ASSERT_NEAR(intersection.x(), a.x(), 1e-6);
//		ASSERT_NEAR(intersection.y(), a.y(), 1e-6);
//		ASSERT_NEAR(intersection.z(), a.z(), 1e-6);
//
//	}
//
//}

TEST(longitude_sweep_tests, test_signed_longitude_difference)
{

	random_numbers::RandomNumberGenerator rng(42);

	// For 1000 iterations...
	for (int i = 0; i < 1000; ++i)
	{
		// Construct a random unit vector:
		math::Vec3d a(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
		a = a.normalized();

		// Generate an angle.
		double angle = rng.uniformReal(-M_PI, M_PI);

		// Rotate a around the vertical axis by the angle.
		math::Vec3d b = math::Quaterniond::fromAxisAngle(math::Vec3d(0, 0, 1), angle).rotate(a);

		// Grab the longitude of a and b.
		double a_lon = longitude(a, math::Vec3d(0, 0, 0));
		double b_lon = longitude(b, math::Vec3d(0, 0, 0));

		// compute the signed difference.
		double signed_diff = signed_longitude_difference(b_lon, a_lon);

		ASSERT_NEAR(signed_diff, angle, 1e-6);
	}

}

TEST(longitude_sweep_tests, signed_longitude_difference) {


	random_numbers::RandomNumberGenerator rng(42);

	// For 1000 iterations...
	for (int i = 0; i < 1000; ++i) {
		// Construct a random unit vector:
		math::Vec3d a(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
		a = a.normalized();

		// Generate an angle.
		double angle = rng.uniformReal(-M_PI, M_PI);

		double signed_angle = rng.uniformReal(-M_PI, M_PI);

		double angle2 = wrap_angle(angle + signed_angle);

		double signed_diff = signed_longitude_difference(angle2, angle);

		ASSERT_NEAR(signed_diff, signed_angle, 1e-6);
	}

}

TEST(longitude_sweep_tests, longitude_interpolation) {

	// Pick two random longitudes in the [-pi,pi] range:
	random_numbers::RandomNumberGenerator rng(42);

	// Repeat 1000 times:
	for (int i = 0; i < 1000; ++i) {

		double lon1 = rng.uniformReal(-M_PI, M_PI);
		double lon2 = rng.uniformReal(-M_PI, M_PI);

		// Pick a random t in the [0,1] range:
		double t = rng.uniformReal(0, 1);

		// Compute the interpolated longitude:
		LongitudeRange range(lon1, lon2);

		double lon = range.interpolate(t).longitude;

		ASSERT_LE(-M_PI, lon);
		ASSERT_LE(lon, M_PI);

		// Do reverse-interpolation to see if we get t back.
		ASSERT_NEAR(range.reverse_interpolate(lon), t, 1e-6);
	}

}

TEST(longitude_sweep_tests, intersection) {

	// Generate four random points on the unit sphere:
	random_numbers::RandomNumberGenerator rng(42);

	for (int i = 0; i < 10000; ++i) {

		RelativeVertex construction_center {
				.longitude = rng.uniformReal(-M_PI, M_PI),
				.latitude = rng.uniformReal(-M_PI / 2.0, M_PI / 2.0)
		};

		// Construct two edges that intersect at the expected intersection, with correct ordering:
		RelativeVertex a1 {
				wrap_angle(construction_center.longitude - rng.uniformReal(0.0, M_PI / 2.0)),
				rng.uniformReal(-M_PI / 2.0, construction_center.latitude)
		};

		RelativeVertex a2 {
				wrap_angle(construction_center.longitude + rng.uniformReal(0.0, M_PI / 2.0)),
				rng.uniformReal(construction_center.latitude, M_PI / 2.0)
		};

		RelativeVertex b1 {
				wrap_angle(construction_center.longitude - rng.uniformReal(0.0, M_PI / 2.0)),
				rng.uniformReal(construction_center.latitude, M_PI / 2.0)
		};

		RelativeVertex b2 {
				wrap_angle(construction_center.longitude + rng.uniformReal(0.0, M_PI / 2.0)),
				rng.uniformReal(-M_PI / 2.0, construction_center.latitude)
		};

		// Construct the edges:
		OrderedArcEdge A1 { a1, a2 };
		OrderedArcEdge B1 { b1, b2 };

		// Geogebra:
//		std::cerr << "C = (" << construction_center.longitude << "," << construction_center.latitude << ")" << std::endl;
		std::cerr << "A = Segment((" << a1.longitude << "," << a1.latitude << "),(" << a2.longitude << "," << a2.latitude << "))" << std::endl;
		std::cerr << "B = Segment((" << b1.longitude << "," << b1.latitude << "),(" << b2.longitude << "," << b2.latitude << "))" << std::endl;

		auto overlap = A1.longitude_range().overlap(B1.longitude_range());

		double lat1 = A1.latitudeAtLongitude(overlap.start);
		double lat2 = B1.latitudeAtLongitude(overlap.start);

		if (lat1 > lat2) {
			std::swap(A1, B1);
		}

		if (A1.crosses(B1, A1.longitude_range().overlap(B1.longitude_range()).start)) {
			// Compute the intersection:
			RelativeVertex intersection = A1.intersection(B1);
			std::cerr << "I = (" << intersection.longitude << "," << intersection.latitude << ")" << std::endl;

			// Check that the intersection is on both edges:
			ASSERT_TRUE(A1.is_on_edge(intersection));
			ASSERT_TRUE(B1.is_on_edge(intersection));
		}
	}

}

TEST(spherical_geometry_test, test_longitude_range) {

	// Get an rng
	random_numbers::RandomNumberGenerator rng(42);

	for (int i = 0; i < 100; ++i) {

		double lon_start = rng.uniformReal(-M_PI, M_PI);
		double range_size = rng.uniformReal(0.0, 2.0 * M_PI);

		LongitudeRange range(lon_start, wrap_angle(lon_start + range_size));

		// Generate a bunch of longitudes and test whether contains works.
		for (int i = 0; i < 10; ++i) {
			double t = rng.uniformReal(0.0, 1.0);

			double lon = range.interpolate(t).longitude;

			ASSERT_TRUE(range.contains(lon));

			ASSERT_NEAR(range.reverse_interpolate(lon), t, 1e-6);
		}

		double remaining_size = 2.0 * M_PI - range_size;

		// And some angles outside:
		for (int i = 0; i < 10; ++i) {
			double t = rng.uniformReal(0.0, 1.0);

			double lon = wrap_angle(range.end + t * remaining_size);

			ASSERT_FALSE(range.contains(lon));
		}
	}
}

TEST(spherical_geometry_test, longitude_range_overlap) {

	// Generate a bunch of overlapping longitude ranges and check that the overlap test returns true.

	random_numbers::RandomNumberGenerator rng(42);

	for (int i = 0; i < 100; ++i) {

		double lon_start = rng.uniformReal(-M_PI, M_PI);
		double range_size = rng.uniformReal(0.0, 2.0 * M_PI);

		LongitudeRange range1(lon_start, wrap_angle(lon_start + range_size));

		double t = rng.uniform01();

		double lon2_start = range1.interpolate(t).longitude;
		double rg2_size = rng.uniformReal(0.0, 2.0 * M_PI);

		LongitudeRange range2(lon2_start, wrap_angle(lon2_start + rg2_size));

		// Make sure they do overlap.
		ASSERT_TRUE(range1.overlaps(range2));

		// Swap them and test again.
		ASSERT_TRUE(range2.overlaps(range1));
	}

}

TEST(spherical_geometry_test, longitude_range_overlap_negative) {

	random_numbers::RandomNumberGenerator rng(42);

	// Generate non-overlapping ranges and test that their overlap test returns false.
	for (int i = 0; i < 100; ++i) {

		double lon_start = rng.uniformReal(-M_PI, M_PI);
		double range_size = rng.uniformReal(0.0, 2.0 * M_PI);

		LongitudeRange range1(lon_start, wrap_angle(lon_start + range_size));

		double remaining = 2.0 * M_PI - range_size;

		double range2_size = rng.uniformReal(0.0, remaining);

		remaining -= range2_size;

		// Put the range2 start in the remaining space somewhere

		double range2_start = wrap_angle(range1.end + rng.uniformReal(0.0, remaining));

		LongitudeRange range2(range2_start, wrap_angle(range2_start + range2_size));

		// Make sure they do not overlap.
		ASSERT_FALSE(range1.overlaps(range2));

		// Swap them and test again.
		ASSERT_FALSE(range2.overlaps(range1));

	}

}

TEST(spherical_geometry_test, longitude_range_clamp) {

	random_numbers::RandomNumberGenerator rng(42);

	for (int i = 0; i < 100; ++i) {

		double range_start = rng.uniformReal(-M_PI, M_PI);
		double range_size = rng.uniformReal(0.0, 2.0 * M_PI);

		LongitudeRange range(range_start, wrap_angle(range_start + range_size));

		// Pick a random longitude in the range:
		double lon = range.interpolate(rng.uniform01()).longitude;

		// Clamping should yield it right back:
		ASSERT_NEAR(range.clamp(lon), lon, 1e-6);

		double remaining = 2.0 * M_PI - range_size;

		// Pick something before halfway inside the remaining:
		double lon_before = wrap_angle(range.end + rng.uniformReal(0.0, remaining / 2.0));

		std::cout << "Lon before: " << lon_before << std::endl;
		std::cout << "Range: " << range.start << " -> " << range.end << std::endl;
		std::cout << "Clamped: " << range.clamp(lon_before) << std::endl;

		// Check that it clamps to the end:
		ASSERT_EQ(range.clamp(lon_before), range.end);

		// After halfway:
		double lon_after = wrap_angle(range.start - rng.uniformReal(0.0, remaining / 2.0));

		// Check that it clamps to the start:
		ASSERT_EQ(range.clamp(lon_after), range.start);

	}

}
