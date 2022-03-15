#ifndef NEW_PLANNERS_MOVEITHIGHWAY_H
#define NEW_PLANNERS_MOVEITHIGHWAY_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include "procedural_tree_generation.h"
#include "Highway.h"

class CollisionShell {

public:

    virtual moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone,
                                                    const Apple &a) = 0;

    virtual std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone,
                                                                const Apple &a,
                                                                const Apple &b) = 0;

};

class SphereShell : public CollisionShell {

    Eigen::Vector3d center;
public:
    SphereShell(Eigen::Vector3d center, double radius);

private:
    double radius;

public:
    moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) override;

    std::vector<moveit::core::RobotState>
    path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) override;

};

class SphereShellHighway : public Highway<Apple> {

    SphereShell sphere_shell;
    ompl::base::SpaceInformationPtr si;
public:
    explicit SphereShellHighway(SphereShell sphereShell, ompl::base::SpaceInformationPtr si);

public:
    ompl::geometric::PathGeometric plan_highway(const Apple &a, const Apple &b) override;

    void on_ramp(const Apple &a, ompl::base::State *result) override;
};

#endif //NEW_PLANNERS_MOVEITHIGHWAY_H
