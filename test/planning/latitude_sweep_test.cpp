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

TEST(longitude_sweep_tests, intersection_longitude_test)
{

    random_numbers::RandomNumberGenerator rng(42);

    // For 1000 iterations...
    for (int i = 0; i < 1000; ++i)
    {

        // Construct a random unit vector:
        math::Vec3d a(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
        a = a.normalized();

        Edge A1 = randomEdgeContainingPoint(a, rng);
        Edge B1 = randomEdgeContainingPoint(a, rng);

        math::Vec3d intersection = Edge_intersection(A1, B1);

        ASSERT_NEAR(intersection.x(), a.x(), 1e-6);
        ASSERT_NEAR(intersection.y(), a.y(), 1e-6);
        ASSERT_NEAR(intersection.z(), a.z(), 1e-6);

    }

}

TEST(longitude_sweep_tests, signed_longitude_difference)
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

TEST(longitude_sweep_tests, sweepline_latitude_intersection_test)
{
    random_numbers::RandomNumberGenerator rng(42);

    // For 1000 iterations...
    for (int i = 0; i < 1000; ++i)
    {
        // Construct a random unit vector:
        math::Vec3d a(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
        a = a.normalized();

        Edge A1 = randomEdgeContainingPoint(a, rng);

        double a_lon = longitude(a, math::Vec3d(0, 0, 0));

        double lat = latitude(A1, a_lon);

        // Reconstruct the point from the latitude and longitude:
        math::Vec3d reconstructed_point(
            std::cos(lat) * std::cos(a_lon),
            std::cos(lat) * std::sin(a_lon),
            std::sin(lat)
        );

        ASSERT_NEAR(reconstructed_point.x(), a.x(), 1e-6);
        ASSERT_NEAR(reconstructed_point.y(), a.y(), 1e-6);
        ASSERT_NEAR(reconstructed_point.z(), a.z(), 1e-6);

    }

    // Regressions:

    // Edge: (0.115449, -0.214643, -0.00318132) -> (0.129188, -0.238531, 0.0697457)
    // At longitude: -1.56451

    Edge A1 {
        {
            math::Vec3d(0.115449, -0.214643, -0.00318132),
            math::Vec3d(0.129188, -0.238531, 0.0697457)
        }
    };

    double a_lon = -1.56451;

    latitude(A1, a_lon);

}
