
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>

robowflex::Trajectory visitAllKnn(const std::vector<Apple> &apples,
                                  const moveit::core::RobotState& start_state,
                                  const size_t k,
                                  const robowflex::SceneConstPtr &scene,
                                  const robowflex::RobotConstPtr &robot) {

    ompl::NearestNeighborsGNAT<Eigen::Vector3d> unvisited_nn;

    for (const Apple& apple: apples) {
        unvisited_nn.add(apple.center);
    }

    Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    ompl_interface::ModelBasedStateSpaceSpecification spec(robot->getModel(), "whole_body");

    auto state_space = std::make_shared<CustomModelBasedStateSpace>(spec);
    state_space->setStateSamplerAllocator([](const ompl::base::StateSpace *space) {
        return std::make_shared<DroneStateSampler>(space);
    });

    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);
    si->setStateValidityChecker(std::make_shared<StateValidityChecker>(si.get(), scene));
    si->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(si.get(), robot, scene));
    si->setup();

    full_trajectory.addSuffixWaypoint(genStartState(robot));

    ompl::geometric::PRM prm(si);

    auto avoid_branches = std::make_shared<InverseClearanceIntegralObjectiveOMPL>(si, false);

    LeavesCollisionChecker leaves_collision_data(tree_scene.leaf_vertices);

    Json::Value root;

    for (const Apple &apple: tree_scene.apples) {

        Json::Value apple_statpoint;

        apple_statpoint["apple"][0] = apple.center.x();
        apple_statpoint["apple"][1] = apple.center.y();
        apple_statpoint["apple"][2] = apple.center.z();

        ompl::base::ScopedState start(si);
        state_space->copyToOMPLState(start.get(), full_trajectory.getTrajectory()->getLastWayPoint());

        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef->addStartState(start);
//        pdef->setOptimizationObjective(avoid_branches);

        auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.2, apple.center);
        pdef->setGoal(goal);

        prm.setProblemDefinition(pdef);

        std::cout << "Planner start:" << std::endl;

        collision_detection::AllowedCollision::Type allowed_type;
        bool is_allowed = scene->getACMConst().getAllowedCollision("leaves", "base_link", allowed_type);

        std::cout << "Allowded: " << is_allowed << std::endl;

        scene->getACMConst().print(std::cout);

        scene->getACMConst().getDefaultEntry("leaves", allowed_type);
        std::cout << "Allowded: " << is_allowed << std::endl;

        std::cout << std::endl;

        std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();
        ompl::base::PlannerStatus status = prm.solve(ompl::base::timedPlannerTerminationCondition(5.0));
        std::chrono::steady_clock::time_point post_solve = std::chrono::steady_clock::now();


        long elapsed_millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                (post_solve - pre_solve)).count();

        ompl::geometric::PathSimplifier ps(si);

        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {

            auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

            ps.shortcutPath(*path, 50);
            ps.smoothBSpline(*path);

            for (auto state: path->getStates()) {
                state_space->copyToRobotState(*robot->getScratchState(), state);
                full_trajectory.addSuffixWaypoint(*robot->getScratchState());

                Json::Value traj_pt;
                for (int i = 0; i < robot->getScratchState()->getVariableCount(); i += 1) {
                    traj_pt["values"][i] = robot->getScratchState()->getVariablePosition(i);
                }
//                traj_pt["clearance"] = si->getStateValidityChecker()->clearance(state);
                apple_statpoint["trajectory"].append(traj_pt);

                //leaves_collision_data.checkLeafCollisions(*robot->getScratchState());
            }

            std::cout << "Point-to-point solution found in " << elapsed_millis << "ms" << std::endl;

            apple_statpoint["solved"] = true;

        } else {
            apple_statpoint["solved"] = false;
            std::cout << "Apple unreachable" << std::endl;
        }

        apple_statpoint["feasible_solve_milliseconds"] = (int) elapsed_millis;
        apple_statpoint["prm_nodes_after_solve"] = (int) prm.milestoneCount();
        apple_statpoint["prm_edges_after_solve"] = (int) prm.edgeCount();
        apple_statpoint["goal_samples_tried"] = (int) goal->getSamplesTried();
        apple_statpoint["goal_samples_yielded"] = (int) goal->getSamplesYielded();

        root.append(apple_statpoint);

        prm.clearQuery();
    }

}