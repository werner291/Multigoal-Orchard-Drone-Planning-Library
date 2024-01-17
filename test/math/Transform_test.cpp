// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 17-1-24.
//

#include <gtest/gtest.h>
#include <random_numbers/random_numbers.h>
#include "../../src/math/Vec3.h"
#include "../../src/math/Transform.h"

using namespace mgodpl;

TEST(Transform_test, inverse_randomized)
{

    // Get a random generator.
    random_numbers::RandomNumberGenerator generator(42);

    for (int i = 0; i < 100; ++i)
    {
        // Generate a random translation and unit quaternion
        math::Vec3d translation = {
            generator.uniformReal(-10.0, 10.0),
            generator.uniformReal(-10.0, 10.0),
            generator.uniformReal(-10.0, 10.0)
        };

        math::Quaterniond q = math::Quaterniond::fromAxisAngle(
            math::Vec3d{
                generator.uniformReal(-1.0, 1.0),
                generator.uniformReal(-1.0, 1.0),
                generator.uniformReal(-1.0, 1.0)
            }.normalized(),
            generator.uniformReal(-M_PI, M_PI)
        );

        math::Transformd tf {
            .translation = translation,
            .orientation = q
        };

        math::Transformd tf_inv = tf.inverse();

        // Applying the tf then tf_inv should give the identity transform. Test it with 10 points.
        for (int j = 0; j < 10; ++j)
        {
            math::Vec3d pt = {
                generator.uniformReal(-10.0, 10.0),
                generator.uniformReal(-10.0, 10.0),
                generator.uniformReal(-10.0, 10.0)
            };

            math::Vec3d pt_transformed = tf.apply(pt);
            math::Vec3d pt_transformed_then_inverted = tf_inv.apply(pt_transformed);

            ASSERT_NEAR(pt.x(), pt_transformed_then_inverted.x(), 1e-6);
            ASSERT_NEAR(pt.y(), pt_transformed_then_inverted.y(), 1e-6);
            ASSERT_NEAR(pt.z(), pt_transformed_then_inverted.z(), 1e-6);
        }
    }


}