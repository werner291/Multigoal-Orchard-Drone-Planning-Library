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

TEST(longitude_sweep_tests, intersection_longitude_test)
{

    random_numbers::RandomNumberGenerator rng(42);

    // For 1000 iterations...
    for (int i = 0; i < 1000; ++i)
    {

        // Construct a random unit vector:
        math::Vec3d a(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
        a = a.normalized();

        // Construct two perpendicular vectors to A.
        math::Vec3d nB = a.cross(math::Vec3d(rng.gaussian01(), rng.gaussian01(), rng.gaussian01())).normalized();
        math::Vec3d nC = a.cross(math::Vec3d(rng.gaussian01(), rng.gaussian01(), rng.gaussian01())).normalized();

        // Use NB and NC as rotation axes to construct two pairs of points where the geodesic intersects at a.

        double angleA1 = rng.uniformReal(-M_PI / 2.0, 0.0);
        double angleA2 = rng.uniformReal(0.0, M_PI / 2.0);

        double angleB1 = rng.uniformReal(-M_PI / 2.0, 0.0);
        double angleB2 = rng.uniformReal(0.0, M_PI / 2.0);

        Edge A1 {
            {
                math::Quaterniond::fromAxisAngle(nB, angleA1).rotate(a),
                math::Quaterniond::fromAxisAngle(nB, angleA2).rotate(a)
            }
        };

        Edge B1 {
            {
                math::Quaterniond::fromAxisAngle(nC, angleB1).rotate(a),
                math::Quaterniond::fromAxisAngle(nC, angleB2).rotate(a)
            }
        };

        math::Vec3d intersection = intersection_longitude(A1, B1);

        ASSERT_NEAR(intersection.x(), a.x(), 1e-6);
        ASSERT_NEAR(intersection.y(), a.y(), 1e-6);
        ASSERT_NEAR(intersection.z(), a.z(), 1e-6);

    }

}