// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 17-1-24.
//

#include "RelativeLatLonGrid.h"
#include <Eigen/Geometry>

namespace mgodpl
{
    RelativeLatLonGrid from_center_and_ideal_vector(const math::Vec3d& center, const math::Vec3d& ideal_vector)
    {
        // Compute a rotation matrix such that the ideal vector is the unit X axis.
        auto q = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d(1.0, 0.0, 0.0),
            Eigen::Vector3d(ideal_vector.x(), ideal_vector.y(), ideal_vector.z())
        );

        math::Transformd to_global {
            .translation = center,
            .orientation = math::Quaterniond{q.x(), q.y(), q.z(), q.w()}
        };

        return RelativeLatLonGrid{
            .grid = LatLonGrid{
                {-M_PI / 2.0, M_PI / 2.0},
                {-M_PI, M_PI},
                0.025,
                25,
                25
            },
            .transform = to_global.inverse(),
            .to_global = to_global
        };
    }

    void insert_triangle(RelativeLatLonGrid& grid, const mgodpl::Triangle& triangle)
    {
        // Transform the triangle into the grid's coordinate system.
        mgodpl::Triangle transformed_triangle{
            .vertices = {
                grid.transform.apply(triangle.vertices[0]),
                grid.transform.apply(triangle.vertices[1]),
                grid.transform.apply(triangle.vertices[2])
            }
        };

        grid.grid.insert_triangle(transformed_triangle);
    }
}
