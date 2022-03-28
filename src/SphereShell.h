#ifndef NEW_PLANNERS_SPHERESHELL_H
#define NEW_PLANNERS_SPHERESHELL_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include "procedural_tree_generation.h"
#include "moveit_conversions.h"

class SphereShell {

    Eigen::Vector3d center;
public:
    const Eigen::Vector3d &getCenter() const;

    double getRadius() const;

private:
    double radius;

public:
    SphereShell(Eigen::Vector3d center, double radius);

    [[nodiscard]] moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const;

    [[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const;

    [[nodiscard]] Eigen::Vector3d applePositionOnSphere(const Apple &a) const;
};

class OMPLSphereShellWrapper {
    SphereShell shell;
public:
    const SphereShell &getShell() const;

private:
    ompl::base::SpaceInformationPtr si;
public:
    OMPLSphereShellWrapper(SphereShell shell, ompl::base::SpaceInformationPtr si);

    ompl::base::ScopedStatePtr state_on_shell(const Apple& apple) const;

    ompl::geometric::PathGeometric path_on_shell(const Apple& a, const Apple& b);


};

#endif //NEW_PLANNERS_SPHERESHELL_H
