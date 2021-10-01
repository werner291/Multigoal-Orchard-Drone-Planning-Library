#ifndef NEW_PLANNERS_POINTTOPOINTPLANNER_H
#define NEW_PLANNERS_POINTTOPOINTPLANNER_H

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/robot.h>
#include <json/value.h>
#include "../procedural_tree_generation.h"
#include "../ompl_custom.h"
#include "../LeavesCollisionChecker.h"

struct PointToPointPlanResult {
    double solution_length{};
    robowflex::Trajectory point_to_point_trajectory;
    Eigen::Vector3d endEffectorTarget;
};


class PointToPointPlanner {

    const std::shared_ptr<robowflex::Robot> robot_;
    const ompl::base::PlannerPtr planner_;
    const std::shared_ptr<ompl::base::OptimizationObjective> optimizationObjective_;

public:
    PointToPointPlanner(const ompl::base::PlannerPtr &planner,
                        const std::shared_ptr<ompl::base::OptimizationObjective> &optimizationObjective,
                        const std::shared_ptr<robowflex::Robot> robot);

    std::optional<PointToPointPlanResult> planPointToPoint(const moveit::core::RobotState &from_state,
                                                           const std::vector<Eigen::Vector3d> &targets,
                                                           double maxTime);

    std::optional<PointToPointPlanResult> planPointToPoint(const moveit::core::RobotState &from_state,
                                                           const Eigen::Vector3d& target);

    ompl::base::GoalPtr constructUnionGoal(const std::vector<Eigen::Vector3d>& targets);

};


#endif //NEW_PLANNERS_POINTTOPOINTPLANNER_H
