#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <robowflex_library/trajectory.h>

#include <random>
#include "multi_goal_planners.h"
#include "ompl_custom.h"
#include "LeavesCollisionChecker.h"
#include "procedural_tree_generation.h"
#include "json_utils.h"
#include "UnionGoalSampleableRegion.h"

std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space);


void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension);


void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension) {
    for (size_t i = 0; i < extension.getNumWaypoints(); i++) {
        full_trajectory.addSuffixWaypoint(extension.getTrajectory()->getWayPoint(i));
    }
}



std::optional<PointToPointPlanResult>
planPointToPoint(const robowflex::RobotConstPtr &robot,
                 ompl::base::Planner &planner,
                 const ompl::base::GoalPtr &goal,
                 const moveit::core::RobotState &from_state) {

    std::optional<PointToPointPlanResult> result;

    robowflex::Trajectory trajectory(robot, "whole_body");

    auto state_space = planner.getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

    ompl::base::ScopedState start(planner.getSpaceInformation());
    state_space->copyToOMPLState(start.get(), from_state);

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner.getSpaceInformation());
    pdef->addStartState(start.get());

    pdef->setGoal(goal);

    planner.setProblemDefinition(pdef);
    if (!planner.isSetup()) planner.setup(); // We explicitly do the setup beforehand to avoid counting it in the benchmarking.

    std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();
    ompl::base::PlannerStatus status = planner.solve(ompl::base::timedPlannerTerminationCondition(0.1));
    std::chrono::steady_clock::time_point post_solve = std::chrono::steady_clock::now();

    long elapsed_millis = std::chrono::duration_cast<std::chrono::milliseconds>(
            (post_solve - pre_solve)).count();

    ompl::geometric::PathSimplifier ps(planner.getSpaceInformation());

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {

        auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

//        ps.shortcutPath(*path, 50);
//        ps.smoothBSpline(*path);

        moveit::core::RobotState st(robot->getModelConst());

//        assert(planner.getSpaceInformation()->checkMotion(path->getStates(),3));

        for (auto state: path->getStates()) {
            state_space->copyToRobotState(st, state);
            trajectory.addSuffixWaypoint(st);
        }

        result = {
                .solution_length = path->length(),
                .point_to_point_trajectory = trajectory
        };
    } else {
        result = {};
        std::cout << "Apple unreachable" << std::endl;
    }

    planner.clearQuery();

    return result;
}

KNNPlanner::KNNPlanner(size_t k) : k(k) {}

MultiGoalPlanResult KNNPlanner::plan(const std::vector<Apple> &apples, const moveit::core::RobotState &start_state,
                                     const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                                     ompl::base::Planner &point_to_point_planner) {
    ompl::NearestNeighborsGNAT<Eigen::Vector3d> unvisited_nn;
    unvisited_nn.setDistanceFunction([](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return (a - b).norm();
    });

    for (const Apple &apple: apples) {
        unvisited_nn.add(apple.center);
    }

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    while (unvisited_nn.size() > 0) {

        const Eigen::Vector3d start_eepos = full_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                "end_effector").translation();

        std::vector<Eigen::Vector3d> knn;
        unvisited_nn.nearestK(start_eepos, k, knn);

        std::optional<PointToPointPlanResult> bestResult;
        double best_length = INFINITY;
        Eigen::Vector3d best_target;

        for (const auto &target: knn) {
            auto subgoal = std::make_shared<DroneEndEffectorNearTarget>(
                    point_to_point_planner.getSpaceInformation(), 0.2,
                    target);

            auto pointToPointResult = planPointToPoint(robot,
                                                       point_to_point_planner,
                                                       subgoal,
                                                       full_trajectory.getTrajectory()->getLastWayPoint());

            if (pointToPointResult.has_value() && pointToPointResult.value().solution_length < best_length) {
                bestResult = pointToPointResult;
                best_length = pointToPointResult.value().solution_length;
                best_target = target;
            }
        }

        if (bestResult.has_value()) {
            unvisited_nn.remove(best_target);
            extendTrajectory(full_trajectory, bestResult.value().point_to_point_trajectory);
            root["segments"].append(makePointToPointJson(best_target, bestResult));
        } else {
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }


    }

    std::ostringstream stringStream;
    stringStream << k;
    stringStream << "-NN";

    root["ordering"] = stringStream.str();

    return {full_trajectory, root};
}

UnionKNNPlanner::UnionKNNPlanner(size_t k) : k(k) {}

MultiGoalPlanResult UnionKNNPlanner::plan(const std::vector<Apple> &apples, const moveit::core::RobotState &start_state,
                                          const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                                          ompl::base::Planner &point_to_point_planner) {
    ompl::NearestNeighborsGNAT<Eigen::Vector3d> unvisited_nn;
    unvisited_nn.setDistanceFunction([](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return (a - b).norm();
    });

    for (const Apple &apple: apples) {
        unvisited_nn.add(apple.center);
    }

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    while (unvisited_nn.size() > 0) {

        const Eigen::Vector3d start_eepos = full_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                "end_effector").translation();

        std::vector<Eigen::Vector3d> knn;
        unvisited_nn.nearestK(start_eepos, k, knn);

        std::vector<std::shared_ptr<const ompl::base::GoalSampleableRegion>> subgoals;

        for (const auto &target: knn) {
            subgoals.push_back(std::make_shared<DroneEndEffectorNearTarget>(
                    point_to_point_planner.getSpaceInformation(), 0.2,
                    target));
        }

        auto pointToPointResult = planPointToPoint(robot,
                                                   point_to_point_planner,
                                                   std::make_shared<UnionGoalSampleableRegion>(
                                                           point_to_point_planner.getSpaceInformation(), subgoals),
                                                   full_trajectory.getTrajectory()->getLastWayPoint());

        if (pointToPointResult.has_value()) {

            auto traj = pointToPointResult.value().point_to_point_trajectory.getTrajectory();

            const Eigen::Vector3d end_eepos = pointToPointResult.value().point_to_point_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                    "end_effector").translation();

            bool which_target = false;
            for (auto &tgt: knn) {
                if ((tgt - end_eepos).norm() < 0.2) {
                    unvisited_nn.remove(tgt);
                    extendTrajectory(full_trajectory, pointToPointResult.value().point_to_point_trajectory);
                    root["segments"].append(makePointToPointJson(tgt, pointToPointResult));
                    which_target = true;
                    break;
                }
            }
            assert(which_target);
        } else {
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }


    }

    root["ordering"] = this->getName();

    return {full_trajectory, root};
}

std::string RandomPlanner::getName() {
    return "Random";
}

MultiGoalPlanResult RandomPlanner::plan(const std::vector<Apple> &apples, const moveit::core::RobotState &start_state,
                                        const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                                        ompl::base::Planner &point_to_point_planner) {

    std::vector<Eigen::Vector3d> targets;

    for (const Apple &apple: apples) {
        targets.push_back(apple.center);
    }

    std::shuffle(targets.begin(), targets.end(), std::mt19937(std::random_device()()));

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    for (const auto &target: targets) {
        auto pointToPointPlanResult = planPointToPoint(robot,
                                                       point_to_point_planner,
                                                       std::make_shared<DroneEndEffectorNearTarget>(
                                                               point_to_point_planner.getSpaceInformation(),
                                                               0.2,
                                                               target),
                                                       full_trajectory.getTrajectory()->getLastWayPoint());


        root["segments"].append(makePointToPointJson(target, pointToPointPlanResult));
    }

    root["ordering"] = "random";

    return {full_trajectory, root};
}
