//
// Created by werner on 12-10-21.
//

#ifndef NEW_PLANNERS_TEST_UTILS_H
#define NEW_PLANNERS_TEST_UTILS_H

#include "../src/ompl_custom.h"

std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone);

std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> genGoals(const ompl::base::SpaceInformationPtr &si);


#endif //NEW_PLANNERS_TEST_UTILS_H
