// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-2-24.
//

#ifndef SCANNING_MOTIONS_H
#define SCANNING_MOTIONS_H
#include <fcl/narrowphase/collision_object.h>

#include "RobotModel.h"
#include "RobotPath.h"

namespace mgodpl
{
    /**
     * @enum Direction
     * @brief An enumeration to represent the direction of scanning motion.
     */
    enum class Direction
    {
        LEFT, ///< Represents the left direction.
        RIGHT ///< Represents the right direction.
    };

    /**
     * @fn RobotPath generateSidewaysScanningMotion(const robot_model::RobotModel& robot, const fcl::CollisionObjectd& treeTrunkObject, const mgodpl::math::Vec3d& fruitCenter, double initialLongitude, double initialLatitude, double scanDistance, Direction direction)
     * @brief Generates a sideways scanning motion for a robot.
     *
     * This function generates a sideways scanning motion for a robot. The motion is generated based on the provided parameters.
     * The direction of the motion can be either left or right, as specified by the Direction enum parameter.
     *
     * The motion is at most half a circle, or until the first collision is detected.
     *
     * @param robot The robot model.
     * @param treeTrunkObject The tree trunk object used for collision detection.
     * @param fruitCenter The center of the fruit to be scanned.
     * @param initialLongitude The initial longitude for the scanning motion.
     * @param initialLatitude The initial latitude for the scanning motion.
     * @param scanDistance The distance to scan from the fruit center.
     * @param direction The direction of the scanning motion (either LEFT or RIGHT).
     *
     * @return A RobotPath object representing the generated scanning motion.
     */
    RobotPath generateSidewaysScanningMotion(const robot_model::RobotModel& robot,
                                             const fcl::CollisionObjectd& treeTrunkObject,
                                             const mgodpl::math::Vec3d& fruitCenter,
                                             double initialLongitude,
                                             double initialLatitude,
                                             double scanDistance,
                                             Direction direction);

    /**
    * @fn RobotPath createLeftRightScanningMotion(const robot_model::RobotModel& robot, const fcl::CollisionObjectd& treeTrunkObject, const mgodpl::math::Vec3d& fruitCenter, double initialLongitude, double initialLatitude, double scanDistance)
    * @brief Creates a left and right scanning motion for a robot, returning to the initial position after.
    *
    * This function creates a left and right scanning motion for a robot. The motion is generated based on the provided parameters.
    * The function first generates a scanning motion to the left, appends it to the path, and then reverses it and appends it again.
    * It then does the same for a scanning motion to the right. The resulting path is returned.
    *
    * @param robot The robot model.
    * @param treeTrunkObject The tree trunk object used for collision detection.
    * @param fruitCenter The center of the fruit to be scanned.
    * @param initialLongitude The initial longitude for the scanning motion.
    * @param initialLatitude The initial latitude for the scanning motion.
    * @param scanDistance The distance to scan from the fruit center.
    *
    * @return A RobotPath object representing the generated scanning motion.
    */
    RobotPath createLeftRightScanningMotion(const robot_model::RobotModel& robot,
                                            const fcl::CollisionObjectd& treeTrunkObject,
                                            const mgodpl::math::Vec3d& fruitCenter,
                                            double initialLongitude,
                                            double initialLatitude,
                                            double scanDistance);
} // namespace mgodpl

#endif //SCANNING_MOTIONS_H
