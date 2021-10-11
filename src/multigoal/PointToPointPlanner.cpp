//
// Created by werner on 10/1/21.
//

#include "PointToPointPlanner.h"
#include "../UnionGoalSampleableRegion.h"
#include "multi_goal_planners.h"


PointToPointPlanner::PointToPointPlanner(const ompl::base::PlannerPtr &planner,
                                         const std::shared_ptr<ompl::base::OptimizationObjective> &optimizationObjective,
                                         const std::shared_ptr<robowflex::Robot> &robot)
        : planner_(planner), optimizationObjective_(optimizationObjective), robot_(robot) {}

std::optional<PointToPointPlanResult>
PointToPointPlanner::planPointToPoint(const moveit::core::RobotState &from_state, const Eigen::Vector3d &target,
                                      double maxTime) {

    // Construct the OMPL goal from the set of targets.
    ompl::base::GoalPtr goal = std::make_shared<DroneEndEffectorNearTarget>(planner_->getSpaceInformation(),
                                                                            GOAL_END_EFFECTOR_RADIUS, target);

    ompl::base::ScopedState start(planner_->getSpaceInformation());
    planner_->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->copyToOMPLState(start.get(), from_state);
    auto plan_result = planToOmplGoal(maxTime, start.get(), goal);

    // "Approximate" solutions can be wildly off, so we accept exact solutions only.
    if (plan_result) {
        auto trajectory = convertTrajectory(*plan_result.value());
        return {PointToPointPlanResult{
                .solution_length = trajectory.getLength(),
                .point_to_point_trajectory = trajectory,
                .endEffectorTarget = target,
        }};
    } else {
        std::cout << "Apple unreachable" << std::endl;
        return {};
    }
}

std::optional<PointToPointPlanResult>
PointToPointPlanner::planToEndEffectorTarget(const moveit::core::RobotState &from_state,
                                             const std::vector<Eigen::Vector3d> &targets,
                                             double maxTime) {

    // Construct the OMPL goal from the set of targets.
    std::vector<std::shared_ptr<const ompl::base::GoalSampleableRegion>> subgoals;

    for (const auto &target: targets) {
        subgoals.push_back(
                std::make_shared<DroneEndEffectorNearTarget>(planner_->getSpaceInformation(),
                                                             GOAL_END_EFFECTOR_RADIUS, target));
    }

    auto goal = std::make_shared<UnionGoalSampleableRegion>(planner_->getSpaceInformation(), subgoals);;

    ompl::base::ScopedState start(planner_->getSpaceInformation());
    planner_->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->copyToOMPLState(start.get(), from_state);
    auto plan_result = planToOmplGoal(maxTime, start.get(), goal);

    // "Approximate" solutions can be wildly off, so we accept exact solutions only.
    if (plan_result) {
        auto trajectory = convertTrajectory(*plan_result.value());
        return {PointToPointPlanResult{
                .solution_length = trajectory.getLength(),
                .point_to_point_trajectory = trajectory,
                .endEffectorTarget = targets[goal->whichSatisfied(plan_result.value()->getStates().back()).value()],
        }};
    } else {
        std::cout << "Apple unreachable" << std::endl;
        return {};
    }
}

std::optional<ompl::geometric::PathGeometric *>
PointToPointPlanner::planToOmplGoal(double maxTime, ompl::base::State *start, const ompl::base::GoalPtr &goal) const {

    planner_->clearQuery();

    auto pdef = constructProblemDefinition(start, goal);
    planner_->setProblemDefinition(pdef);

    // We explicitly do the setup beforehand to avoid counting it in the benchmarking.
    if (!planner_->isSetup()) planner_->setup();

    ompl::base::PlannerStatus status = planner_->solve(ompl::base::timedPlannerTerminationCondition(maxTime));

    // "Approximate" solutions can be wildly off, so we accept exact solutions only.
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        return pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    } else {
        return nullptr;
    }
}

std::shared_ptr<ompl::base::ProblemDefinition>
PointToPointPlanner::constructProblemDefinition(const ompl::base::State *start, const ompl::base::GoalPtr &goal) const {
    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner_->getSpaceInformation());
    pdef->addStartState(start);
    pdef->setOptimizationObjective(optimizationObjective_);
    pdef->setGoal(goal);
    return pdef;
}

robowflex::Trajectory PointToPointPlanner::convertTrajectory(ompl::geometric::PathGeometric &path) {
    // Initialize an empty trajectory.
    robowflex::Trajectory trajectory(robot_, "whole_body");

    moveit::core::RobotState st(robot_->getModelConst());

    auto state_space = planner_->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

    for (auto state: path.getStates()) {
        state_space->copyToRobotState(st, state);
        trajectory.addSuffixWaypoint(st);
    }

    return trajectory;
}

const ompl::base::PlannerPtr &PointToPointPlanner::getPlanner() const {
    return planner_;
}

const std::shared_ptr<ompl::base::OptimizationObjective> &PointToPointPlanner::getOptimizationObjective() const {
    return optimizationObjective_;
}

std::optional<ompl::geometric::PathGeometric *>
PointToPointPlanner::planToOmplState(double maxTime, ompl::base::State *start, const ompl::base::State *goal) const {
    auto gs = std::make_shared<ompl::base::GoalState>(planner_->getSpaceInformation());
    gs->setState(goal);
    return planToOmplGoal(maxTime, start, gs);
}
