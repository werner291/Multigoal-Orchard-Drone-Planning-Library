#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <robowflex_library/trajectory.h>

#include <random>
#include "multi_goal_planners.h"
#include "ompl_custom.h"
#include "LeavesCollisionChecker.h"
#include "procedural_tree_generation.h"
#include "BulletContinuousMotionValidator.h"

Json::Value getStateStatisticsPoint(const moveit::core::RobotState &st);

std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space);


Json::Value eigenToJson(const Eigen::Vector3d &vec) {
    Json::Value apple;
    apple[0] = vec.x();
    apple[1] = vec.y();
    apple[2] = vec.z();
    return apple;
}

MultiGoalPlanResult plan_nn(const std::vector<Apple> &apples,
                            const moveit::core::RobotState &start_state,
                            const robowflex::SceneConstPtr &scene,
                            const robowflex::RobotConstPtr &robot,
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

        const Eigen::Vector3d target = unvisited_nn.nearest(start_eepos);
        unvisited_nn.remove(target);

        auto pointToPointPlanResult = planPointToPoint(robot, full_trajectory,
                                                       point_to_point_planner,
                                                       std::make_shared<DroneEndEffectorNearTarget>(
                                                               point_to_point_planner.getSpaceInformation(),
                                                               0.2,
                                                               target));

        Json::Value json;
        json["apple"] = eigenToJson(target);
        json["solved"] = pointToPointPlanResult.solution_length.has_value();
        if (pointToPointPlanResult.solution_length.has_value()) {
            json["path_length"] = pointToPointPlanResult.solution_length.value();
        }
        root["segments"].append(json);
    }

    root["ordering"] = "NN";

    return {full_trajectory, root};

}

MultiGoalPlanResult plan_knn(const std::vector<Apple> &apples,
                             const moveit::core::RobotState &start_state,
                             const robowflex::SceneConstPtr &scene,
                             const robowflex::RobotConstPtr &robot,
                             size_t k,
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

        PointToPointPlanResult bestResult;
        bestResult.solution_length = {};
        double best_length = INFINITY;
        Eigen::Vector3d best_target;

        for (auto target : knn) {
            auto subgoal = std::make_shared<DroneEndEffectorNearTarget>(
                    point_to_point_planner.getSpaceInformation(), 0.2,
                    target);

            PointToPointPlanResult pointToPointPlanResult = planPointToPoint(robot, full_trajectory,
                                                                             point_to_point_planner,
                                                                             subgoal);

            if (pointToPointPlanResult.solution_length.has_value() && pointToPointPlanResult.solution_length.value() < best_length) {
                bestResult = pointToPointPlanResult;
                best_length = pointToPointPlanResult.solution_length.value();
                best_target = target;
            }
        }

        Json::Value json;
        json["apple"] = eigenToJson(best_target);
        json["solved"] = bestResult.solution_length.has_value();
        if (bestResult.solution_length.has_value()) {
            json["path_length"] = bestResult.solution_length.value();
            unvisited_nn.remove(best_target);
        } else {
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }
        root["segments"].append(json);
    }

    std::ostringstream stringStream;
    stringStream << k;
    stringStream << "-NN";

    root["ordering"] = stringStream.str();

    return {full_trajectory, root};

}

MultiGoalPlanResult plan_k_random(const std::vector<Apple> &apples,
                             const moveit::core::RobotState &start_state,
                             const robowflex::SceneConstPtr &scene,
                             const robowflex::RobotConstPtr &robot,
                             size_t k,
                             ompl::base::Planner &point_to_point_planner) {

    std::vector<Eigen::Vector3d> targets;

    for (const Apple &apple: apples) {
        targets.push_back(apple.center);
    }

    std::shuffle(targets.begin(), targets.end(), std::mt19937(std::random_device()()));

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    while (targets.size() > 0) {

        const Eigen::Vector3d start_eepos = full_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                "end_effector").translation();

        PointToPointPlanResult bestResult;
        bestResult.solution_length = {};
        double best_length = INFINITY;
        Eigen::Vector3d best_target;

        for (int i = 0; i < targets.size() && i < k; i++) {

            Eigen::Vector3d target = targets[i];

            auto subgoal = std::make_shared<DroneEndEffectorNearTarget>(
                    point_to_point_planner.getSpaceInformation(), 0.2,
                    target);

            PointToPointPlanResult pointToPointPlanResult = planPointToPoint(robot, full_trajectory,
                                                                             point_to_point_planner,
                                                                             subgoal);

            if (pointToPointPlanResult.solution_length.has_value() && pointToPointPlanResult.solution_length.value() < best_length) {
                bestResult = pointToPointPlanResult;
                best_length = pointToPointPlanResult.solution_length.value();
                best_target = target;
            }
        }

        Json::Value json;
        json["apple"] = eigenToJson(best_target);
        json["solved"] = bestResult.solution_length.has_value();
        if (bestResult.solution_length.has_value()) {
            json["path_length"] = bestResult.solution_length.value();
            targets.erase(std::find(targets.begin(), targets.end(), best_target));
        } else {
            targets.erase(targets.begin()); // Better picks here? Maybe delete all?
        }
        root["segments"].append(json);
    }

    std::ostringstream stringStream;
    stringStream << k;
    stringStream << "-random";

    root["ordering"] = stringStream.str();

    return {full_trajectory, root};

}

Json::Value makePointToPointJson(const Eigen::Vector3d &target, PointToPointPlanResult &pointToPointPlanResult) {
    Json::Value json;
    json["apple"] = eigenToJson(target);
    json["solved"] = pointToPointPlanResult.solution_length.has_value();
    if (pointToPointPlanResult.solution_length.has_value()) {
        json["path_length"] = pointToPointPlanResult.solution_length.value();
    }
    return json;
}

MultiGoalPlanResult plan_random(const std::vector<Apple> &apples,
                                const moveit::core::RobotState &start_state,
                                const robowflex::SceneConstPtr &scene,
                                const robowflex::RobotConstPtr &robot,
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
        auto pointToPointPlanResult = planPointToPoint(robot, full_trajectory,
                                                       point_to_point_planner,
                                                       std::make_shared<DroneEndEffectorNearTarget>(
                                                               point_to_point_planner.getSpaceInformation(),
                                                               0.2,
                                                               target));


        Json::Value json = makePointToPointJson(target, pointToPointPlanResult);
        root["segments"].append(json);
    }

    root["ordering"] = "random";

    return {full_trajectory, root};

}

Json::Value getStateStatisticsPoint(const moveit::core::RobotState &st) {
    Json::Value traj_pt;
    for (int i = 0; i < st.getVariableCount(); i += 1) {
        traj_pt["values"][i] = st.getVariablePosition(i);
    }

    return traj_pt;
}

PointToPointPlanResult planPointToPoint(const robowflex::RobotConstPtr &robot, robowflex::Trajectory &full_trajectory,
                                        ompl::base::Planner &planner, const ompl::base::GoalPtr &goal) {

    PointToPointPlanResult result;

    auto state_space = planner.getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();
    
    ompl::base::ScopedState start(planner.getSpaceInformation());
    state_space->copyToOMPLState(start.get(), full_trajectory.getTrajectory()->getLastWayPoint());

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

        ps.shortcutPath(*path, 50);
        ps.smoothBSpline(*path);

        moveit::core::RobotState st(robot->getModelConst());

        for (auto state: path->getStates()) {
            state_space->copyToRobotState(st, state);
            full_trajectory.addSuffixWaypoint(st);

            //apple_statpoint["trajectory"].append(getStateStatisticsPoint(st));
        }

//        std::cout << "Point-to-point solution found in " << elapsed_millis << "ms" << std::endl;

        result.solution_length = {path->length()};

    } else {
        result.solution_length = {};
        std::cout << "Apple unreachable" << std::endl;
    }

//    ompl::base::PlannerData pd(planner.getSpaceInformation());
//
//    planner.getPlannerData(pd);

//    apple_statpoint["feasible_solve_milliseconds"] = (int) elapsed_millis;
//    apple_statpoint["prm_nodes_after_solve"] = (int) pd.numVertices();
//    apple_statpoint["prm_edges_after_solve"] = (int) pd.numEdges();
//    apple_statpoint["goal_samples_tried"] = (int) goal->getSamplesTried();
//    apple_statpoint["goal_samples_yielded"] = (int) goal->getSamplesYielded();

    planner.clearQuery();

    return result;
}
