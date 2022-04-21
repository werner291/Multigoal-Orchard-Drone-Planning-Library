#ifndef NEW_PLANNERS_SPHERESHELL_H
#define NEW_PLANNERS_SPHERESHELL_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include "procedural_tree_generation.h"
#include "moveit_conversions.h"

class CollisionFreeShell {

public:
    [[nodiscard]] virtual moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const = 0;

    [[nodiscard]] virtual std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const = 0;

    [[nodiscard]] virtual Eigen::Vector3d applePositionOnShell(const Apple &a) const = 0;

};

class SphereShell : public CollisionFreeShell {

    Eigen::Vector3d center;
public:
    [[nodiscard]] const Eigen::Vector3d &getCenter() const;

    [[nodiscard]] double getRadius() const;

private:
    double radius;

public:
    SphereShell(Eigen::Vector3d center, double radius);

    [[nodiscard]] moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const override;

    [[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const override;

    [[nodiscard]] Eigen::Vector3d applePositionOnShell(const Apple &a) const override;
};

class CylinderShell : public CollisionFreeShell {

    Eigen::Vector2d center;
public:
    CylinderShell(const Eigen::Vector2d &center, double radius);

private:
    double radius;
public:
    [[nodiscard]] const Eigen::Vector2d &getCenter() const;

    [[nodiscard]] double getRadius() const;

private:

public:

    [[nodiscard]] moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const override;

    [[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const override;

    [[nodiscard]] Eigen::Vector3d applePositionOnShell(const Apple &a) const override;
};

class OMPLSphereShellWrapper {
    std::shared_ptr<CollisionFreeShell> shell;
    ompl::base::SpaceInformationPtr si;
public:
    [[nodiscard]] std::shared_ptr<CollisionFreeShell> getShell() const;

public:
    OMPLSphereShellWrapper(std::shared_ptr<CollisionFreeShell> shell, ompl::base::SpaceInformationPtr si);

    void state_on_shell(const Apple& apple, ompl::base::State* st) const;

    ompl::geometric::PathGeometric path_on_shell(const Apple& a, const Apple& b);


};

#endif //NEW_PLANNERS_SPHERESHELL_H
