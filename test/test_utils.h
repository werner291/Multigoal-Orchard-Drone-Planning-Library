
#ifndef NEW_PLANNERS_TEST_UTILS_H
#define NEW_PLANNERS_TEST_UTILS_H

#include "../src/ompl_custom.h"

/**
 * By default, MoveIt's random state generation doesn't randomize the translation
 * component of floating joints.
 *
 * Here, we give it a uniform translation in the [-100.0,100.0] range in every dimension.
 */
std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone);

#endif //NEW_PLANNERS_TEST_UTILS_H
