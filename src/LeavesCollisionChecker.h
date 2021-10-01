//
// Created by werner on 09-09-21.
//

#ifndef NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H
#define NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H

#include <fcl/fcl.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/datastructures/GreedyKCenters.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

class LeavesCollisionChecker {

    fcl::BVHModel<fcl::OBBRSSd> leaves;

public:
    LeavesCollisionChecker(const std::vector<Eigen::Vector3d> &leaf_vertices);

    std::set<size_t> checkLeafCollisions(moveit::core::RobotState &state);

};

class LeavesCollisionCountObjective : public ompl::base::StateCostIntegralObjective {

    std::shared_ptr<moveit::core::RobotModel> robot;
    std::shared_ptr<LeavesCollisionChecker> leaves;

public:
    LeavesCollisionCountObjective(const ompl::base::SpaceInformationPtr &si,
                                  const std::shared_ptr<moveit::core::RobotModel> &robot,
                                  const std::shared_ptr<LeavesCollisionChecker> &leaves);

    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

};

#endif //NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H
