#ifndef NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H
#define NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H

void randomizeUprightWithBase(moveit::core::RobotState &state, double translation_bound);

void moveEndEffectorToGoal(moveit::core::RobotState &state, double tolerance,
                                  const Eigen::Vector3d &target);

#endif //NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H
