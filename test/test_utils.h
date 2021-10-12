//
// Created by werner on 12-10-21.
//

#ifndef NEW_PLANNERS_TEST_UTILS_H
#define NEW_PLANNERS_TEST_UTILS_H

moveit::core::RobotModelPtr loadRobotModel();

std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone);

#endif //NEW_PLANNERS_TEST_UTILS_H
