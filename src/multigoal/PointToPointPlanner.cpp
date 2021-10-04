//
// Created by werner on 10/1/21.
//

#include "PointToPointPlanner.h"
#include "../UnionGoalSampleableRegion.h"


PointToPointPlanner::PointToPointPlanner(const ompl::base::PlannerPtr &planner,
                                         const std::shared_ptr<ompl::base::OptimizationObjective> &optimizationObjective,
                                         const robowflex::RobotPtr robot)
        : planner_(planner), optimizationObjective_(optimizationObjective), robot_(robot) {}

std::optional<PointToPointPlanResult> PointToPointPlanner::planPointToPoint(const moveit::core::RobotState &from_state,
                                                                            const Eigen::Vector3d& target) {

    std::vector<Eigen::Vector3d> targets {target};
    return this->planPointToPoint(from_state, targets, 0);
}

std::optional<PointToPointPlanResult> PointToPointPlanner::planPointToPoint(const moveit::core::RobotState &from_state,
                                                                            const std::vector<Eigen::Vector3d> &targets,
                                                                            double maxTime) {

    ompl::base::GoalPtr goal = this->constructUnionGoal(targets);

    std::optional<PointToPointPlanResult> result;

    robowflex::Trajectory trajectory(robot_, "whole_body");

    auto state_space = planner_->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

    ompl::base::ScopedState start(planner_->getSpaceInformation());
    state_space->copyToOMPLState(start.get(), from_state);
    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner_->getSpaceInformation());
    pdef->addStartState(start.get());
    pdef->setOptimizationObjective(this->optimizationObjective_);
    pdef->setGoal(goal);

    planner_->setProblemDefinition(pdef);

    if (!planner_->isSetup()) planner_->setup(); // We explicitly do the setup beforehand to avoid counting it in the benchmarking.

    std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();
    ompl::base::PlannerStatus status = planner_->solve(ompl::base::timedPlannerTerminationCondition(maxTime));
    std::chrono::steady_clock::time_point post_solve = std::chrono::steady_clock::now();

    long elapsed_millis = std::chrono::duration_cast<std::chrono::milliseconds>(
            (post_solve - pre_solve)).count();

    ompl::geometric::PathSimplifier ps(planner_->getSpaceInformation());

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {

        PointToPointPlanResult actual_result;

        auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

        moveit::core::RobotState st(robot_->getModelConst());

        for (auto state: path->getStates()) {
            state_space->copyToRobotState(st, state);
            trajectory.addSuffixWaypoint(st);
        }

        const Eigen::Vector3d end_eepos = trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform("end_effector").translation();

        bool which_target = false;
        for (auto &tgt: targets) {
            if ((tgt - end_eepos).norm() < 0.1) { // FIXME Don't use magic numbers!
                actual_result.target = tgt;
                which_target = true;
                break;
            }
        }
        assert(which_target);

        actual_result.solution_length = path->length();
        actual_result.point_to_point_trajectory = trajectory;

        return {actual_result};

    } else {
        result = {};
        std::cout << "Apple unreachable" << std::endl;
    }

    planner_->clearQuery();

    return result;

}

ompl::base::GoalPtr PointToPointPlanner::constructUnionGoal(const std::vector<Eigen::Vector3d> &targets) {
    if (targets.size() == 1) {
        return std::make_shared<DroneEndEffectorNearTarget>(planner_->getSpaceInformation(), 0.2, targets[0]);
    } else {
        std::vector<std::shared_ptr<const ompl::base::GoalSampleableRegion>> subgoals;

        for (const auto &target: targets) {
            subgoals.push_back(
                    std::make_shared<DroneEndEffectorNearTarget>(planner_->getSpaceInformation(), 0.2, target));
        }

        return std::make_shared<UnionGoalSampleableRegion>(planner_->getSpaceInformation(), subgoals);
    }
}
