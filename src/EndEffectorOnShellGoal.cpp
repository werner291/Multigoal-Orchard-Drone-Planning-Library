//
// Created by werner on 28-03-22.
//

#include "EndEffectorOnShellGoal.h"
#include "ompl_custom.h"

#include <utility>

EndEffectorOnShellGoal::EndEffectorOnShellGoal(const ompl::base::SpaceInformationPtr &si,
                                               OMPLSphereShellWrapper sphereShell,
                                               Eigen::Vector3d focus)
        : GoalSampleableRegion(si),
          sphereShell(std::move(sphereShell)),
          focus(std::move(focus)) {}

void EndEffectorOnShellGoal::sampleGoal(ompl::base::State *st) const {

    ompl::RNG rng;

    Eigen::Vector3d moved_focus(
            focus.x() + rng.gaussian(0.0, 0.5),
            focus.y() + rng.gaussian(0.0, 0.5),
            focus.z() + rng.gaussian(0.0, 0.5)
    );

    si_->copyState(st, sphereShell.state_on_shell({moved_focus,{0.0,0.0,0.0}})->get());
}

unsigned int EndEffectorOnShellGoal::maxSampleCount() const {
    return INT_MAX;
}

double EndEffectorOnShellGoal::distanceGoal(const ompl::base::State *st) const {
    auto *state_space = si_->getStateSpace()->as<DroneStateSpace>();
    moveit::core::RobotState rs(state_space->getRobotModel());
    state_space->copyToRobotState(rs, st);
    Eigen::Vector3d ee_pos = rs.getGlobalLinkTransform("end_effector").translation();

    return std::abs((ee_pos - sphereShell.getShell().getCenter()).norm() - sphereShell.getShell().getRadius());
}
