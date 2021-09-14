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

struct PointToPointPlanResult {
    Json::Value statPoint;
};

PointToPointPlanResult planPointToPoint(const robowflex::RobotConstPtr &robot, robowflex::Trajectory &full_trajectory,
                                        const std::shared_ptr<DroneStateSpace> &state_space,
                                        const std::shared_ptr<ompl::base::SpaceInformation> &si,
                                        ompl::base::Planner &planner,
                                        const Eigen::Vector3d &target) {

    Json::Value apple_statpoint;

    apple_statpoint["apple"] = eigenToJson(target);

    ompl::base::ScopedState start(planner.getSpaceInformation());
    state_space->copyToOMPLState(start.get(), full_trajectory.getTrajectory()->getLastWayPoint());

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner.getSpaceInformation());
    pdef->addStartState(start.get());

    auto goal = std::make_shared<DroneEndEffectorNearTarget>(planner.getSpaceInformation(), 0.2, target);
    pdef->setGoal(goal);

    planner.setProblemDefinition(pdef);
    planner.setup(); // We explicitly do the setup beforehand to avoid counting it in the benchmarking.

    std::cout << "Planner start:" << std::endl;
    std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();
    ompl::base::PlannerStatus status = planner.solve(ompl::base::timedPlannerTerminationCondition(5.0));
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

            apple_statpoint["trajectory"].append(getStateStatisticsPoint(st));
        }

        std::cout << "Point-to-point solution found in " << elapsed_millis << "ms" << std::endl;

        apple_statpoint["solved"] = true;

    } else {
        apple_statpoint["solved"] = false;
        std::cout << "Apple unreachable" << std::endl;
    }

    ompl::base::PlannerData pd(planner.getSpaceInformation());

    planner.getPlannerData(pd);

    apple_statpoint["feasible_solve_milliseconds"] = (int) elapsed_millis;
    apple_statpoint["prm_nodes_after_solve"] = (int) pd.numVertices();
    apple_statpoint["prm_edges_after_solve"] = (int) pd.numEdges();
    apple_statpoint["goal_samples_tried"] = (int) goal->getSamplesTried();
    apple_statpoint["goal_samples_yielded"] = (int) goal->getSamplesYielded();

    planner.clearQuery();

    return {apple_statpoint};
}


MultiGoalPlanResult plan_nn_rrtconnect(const std::vector<Apple> &apples,
                                       const moveit::core::RobotState &start_state,
                                       const robowflex::SceneConstPtr &scene,
                                       const robowflex::RobotConstPtr &robot) {

    ompl::NearestNeighborsGNAT<Eigen::Vector3d> unvisited_nn;

    for (const Apple &apple: apples) {
        unvisited_nn.add(apple.center);
    }

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    ompl_interface::ModelBasedStateSpaceSpecification spec(robot->getModelConst(), "whole_body");

    auto state_space = std::make_shared<DroneStateSpace>(spec);
    state_space->setStateSamplerAllocator([](const ompl::base::StateSpace *space) {
        return std::make_shared<DroneStateSampler>(space);
    });

    auto si = initSpaceInformation(scene, robot, state_space);

    ompl::geometric::RRTConnect planner(si);

    Json::Value root;

    while (unvisited_nn.size() > 0) {

        const Eigen::Vector3d start_eepos = full_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                "end_effector").translation();

        const Eigen::Vector3d target = unvisited_nn.nearest(start_eepos);
        unvisited_nn.remove(target);

        root.append(planPointToPoint(robot, full_trajectory, state_space, si, planner, target).statPoint);
    }

    return {full_trajectory, root};

}

MultiGoalPlanResult plan_random(const std::vector<Apple> &apples, const moveit::core::RobotState &start_state,
                                const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot) {

    std::vector<Eigen::Vector3d> targets;

    for (const Apple &apple: apples) {
        targets.push_back(apple.center);
    }

    std::shuffle(targets.begin(), targets.end(), std::mt19937(std::random_device()()));

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    ompl_interface::ModelBasedStateSpaceSpecification spec(robot->getModelConst(), "whole_body");

    auto state_space = std::make_shared<DroneStateSpace>(spec);

    auto si = initSpaceInformation(scene, robot, state_space);

    ompl::geometric::PRM planner(si);

    Json::Value root;

    for (const auto &target: targets) {
        root.append(planPointToPoint(robot, full_trajectory, state_space, si, planner, target).statPoint);
    }

    return {full_trajectory, root};

}


std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space) {
    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);
    si->setStateValidityChecker(std::make_shared<StateValidityChecker>(si.get(), scene));
    si->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(si.get(), robot, scene));
    si->setup();

    return si;
}


Json::Value getStateStatisticsPoint(const moveit::core::RobotState &st) {
    Json::Value traj_pt;
    for (int i = 0; i < st.getVariableCount(); i += 1) {
        traj_pt["values"][i] = st.getVariablePosition(i);
    }

    return traj_pt;
}
