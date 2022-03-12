

#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <fstream>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include "../src/planning_scene_diff_message.h"
#include "../src/traveling_salesman.h"
#include "../src/experiment_utils.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"

struct PlanningResult {
    ompl::geometric::PathGeometric path;
    std::set<size_t> goals_visited;
};

PlanningResult planOneByOneInOrder(const std::vector<Apple> &apples, const std::vector<size_t> &ordering_euclidean,
                                                   const std::shared_ptr<ompl::base::SpaceInformation> &si,
                                                   const std::shared_ptr<ompl::base::Planner> &planner,
                                                   const std::shared_ptr<ManipulatorDroneMoveitPathLengthObjective> &objective,
                                                   const ompl::base::State *start_state) {

    ompl::geometric::PathGeometric path(si, start_state);
    std::set<size_t> goals_visited;

    for (size_t next_target : boost::irange<size_t>(0,apples.size()))
    {
        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner->getSpaceInformation());
        pdef->setOptimizationObjective(objective);
        pdef->addStartState(path.getState(path.getStateCount()-1));
        pdef->setGoal(std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apples[ordering_euclidean[next_target]].center));

        planner->setProblemDefinition(pdef);

        if (planner->solve(ompl::base::timedPlannerTerminationCondition(2.0)) ==
            ompl::base::PlannerStatus::EXACT_SOLUTION) {
            path.append(*pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>());
            goals_visited.insert(next_target);
        } else {
            OMPL_WARN("Cannot plan to apple. Skipping...");
        }
    }

    return {path,goals_visited};
}

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    moveit::core::RobotState start_state(drone);

    start_state.setVariablePositions({
        3.0, 3.0, 1.5,      // Position off the side of the tree
        0.0, 0.0, 0.0, 1.0, // Identity rotation
        0.0, 0.0, 0.0, 0.0  // Arm straight out
    });
    start_state.update(true);

    Eigen::Vector3d start_end_effector_pos = start_state.getGlobalLinkTransform("end_effector").translation();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    // FIXME remove this at some point
    std::shuffle(apples.begin(), apples.end(), std::mt19937(std::random_device()()));
//    apples.resize(20);

    const GreatCircleMetric gsm(Eigen::Vector3d(0.0, 0.0, 2.2));

    std::unique_ptr<DistanceHeuristics> heuristics[] = {
            std::make_unique<EuclideanDistanceHeuristics>(start_end_effector_pos),
            std::make_unique<GreatcircleDistanceHeuristics>(start_end_effector_pos, gsm),
    };

    std::unique_ptr<OrderingStrategy> ordering_methods[] = {
            std::make_unique<GreedyOrderingStrategy>(),
            std::make_unique<ORToolsOrderingStrategy>(),
    };

    Json::Value json_values;
    json_values["apples"] = (int) apples.size();

    for (const auto& metric : heuristics) {
        for (const auto& ordering_strategy : ordering_methods) {

            Json::Value json_value;
            json_value["metric"] = metric->name();
            json_value["ordering_strategy"] = ordering_strategy->name();

            auto ordering = ordering_strategy->apple_ordering(apples, *metric);

            json_value["heuristic_cost"] = ordering_heuristic_cost(ordering, apples, *metric);

            ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
            auto state_space = std::make_shared<DroneStateSpace>(spec);
            state_space->setup();
            auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);

            // FIXME see https://github.com/ompl/ompl/issues/885 auto planner = std::make_shared<ompl::geometric::AITstar>(si);
            auto planner = std::make_shared<ompl::geometric::PRMstar>(si);
            auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

            ompl::base::ScopedState start(si);
            state_space->copyToOMPLState(start.get(), start_state);
            assert(si->isValid(start.get()));
            assert(state_space->satisfiesBounds(start.get()));

            auto result = planOneByOneInOrder(apples,ordering,si,planner,objective,start.get());

            json_value["goals_visited"] = (int) result.goals_visited.size();
            json_value["trajectory_length"] = result.path.length();

            json_values["attempts"].append(json_value);
        }
    }

    std::ofstream ofs;
    ofs.open("analysis/preorder_experiment_results.json");
    ofs << json_values;
    ofs.close();


//
//    auto result_path = planOneByOneInOrder(apples, ordering_euclidean, si, planner, objective, start.get());

    return 0;
}


