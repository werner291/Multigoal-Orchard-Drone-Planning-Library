// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-2-24.
//

#include "scanning_motions.h"

#include "collision_detection.h"
#include "spherical_geometry.h"
#include "state_tools.h"

mgodpl::RobotPath mgodpl::generateSidewaysScanningMotion(const robot_model::RobotModel& robot,
                                                         const fcl::CollisionObjectd& treeTrunkObject,
                                                         const mgodpl::math::Vec3d& fruitCenter,
                                                         double initialLongitude,
                                                         double initialLatitude, double scanDistance,
                                                         Direction direction)
{
    const int MAX_ITERATIONS = 32; ///< The maximum number of iterations for the scanning motion.
    double STEP_SIZE = M_PI / (double)MAX_ITERATIONS; ///< The step size for each iteration of the scanning motion.

    // Adjust the step size based on the direction
    if (direction == Direction::LEFT)
    {
        STEP_SIZE = -STEP_SIZE;
    }

    std::vector<RobotState> scanningMotionStates;
    ///< A vector to store the states of the robot during the scanning motion.
    for (int i = 1; i < MAX_ITERATIONS; ++i)
    {
        auto relativeVertex = spherical_geometry::RelativeVertex{
            .longitude = initialLongitude + i * STEP_SIZE, .latitude = initialLatitude
        }.to_cartesian();

        // If collision-free, add to the list:
        if (!check_robot_collision(robot, treeTrunkObject, fromEndEffectorAndVector(robot,
                                       fruitCenter + relativeVertex.normalized() * scanDistance,
                                       relativeVertex)))
        {
            scanningMotionStates.push_back(fromEndEffectorAndVector(robot,
                                                                    fruitCenter + relativeVertex.normalized() *
                                                                    scanDistance,
                                                                    relativeVertex));
        }
        else
        {
            // Stop generating states after the first collision
            break;
        }
    }
    return RobotPath{scanningMotionStates}; ///< Return the generated scanning motion as a RobotPath object.
}

mgodpl::RobotPath mgodpl::createLeftRightScanningMotion(const robot_model::RobotModel& robot,
                                                        const fcl::CollisionObjectd& treeTrunkObject,
                                                        const mgodpl::math::Vec3d& fruitCenter, double initialLongitude,
                                                        double initialLatitude, double scanDistance)
{
    RobotPath path; ///< The path object to store the generated scanning motion.

    // Generate a few additional samples by increasing the long angle:
    RobotPath scan_motion = generateSidewaysScanningMotion(
        robot,
        treeTrunkObject,
        fruitCenter,
        initialLongitude,
        initialLatitude,
        scanDistance,
        Direction::LEFT
    );

    // Append the scan motion to the path.
    path.states.insert(path.states.end(), scan_motion.states.begin(), scan_motion.states.end());
    // And back:
    path.states.insert(path.states.end(), scan_motion.states.rbegin(), scan_motion.states.rend());

    // Same but to the right now:
    RobotPath scan_motion_right = generateSidewaysScanningMotion(
        robot,
        treeTrunkObject,
        fruitCenter,
        initialLongitude,
        initialLatitude,
        scanDistance,
        Direction::RIGHT
    );

    path.states.insert(path.states.end(), scan_motion_right.states.begin(), scan_motion_right.states.end());
    path.states.insert(path.states.end(), scan_motion_right.states.rbegin(), scan_motion_right.states.rend());

    return path; ///< Return the generated scanning motion as a RobotPath object.
}
