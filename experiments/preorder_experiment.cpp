

#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <fstream>

#include "../src/planning_scene_diff_message.h"
#include "../src/traveling_salesman.h"
#include "../src/experiment_utils.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"

ompl::geometric::PathGeometric planOneByOneInOrder(const std::vector<Apple> &apples, const std::vector<size_t> &ordering_euclidean,
                                                   const std::shared_ptr<ompl::base::SpaceInformation> &si,
                                                   std::shared_ptr<ompl::geometric::AITstar> &planner,
                                                   const std::shared_ptr<ManipulatorDroneMoveitPathLengthObjective> &objective,
                                                   const ompl::base::State *start_state) {

    ompl::geometric::PathGeometric path(si, start_state);

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
        } else {
            OMPL_WARN("Cannot plan to apple. Skipping...");
        }
    }

    return path;
}

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    moveit::core::RobotState start_state(drone);

    start_state.setVariablePosition(0, 3.0);
    start_state.setVariablePosition(1, 3.0);
    start_state.setVariablePosition(2, 1.5);

    start_state.update(true);

    Eigen::Vector3d start_end_effector_pos = start_state.getGlobalLinkTransform("end_effector").translation();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    struct Metrics {
        const std::string name;
        const std::function<double(const Apple &)> start_to_apple;
        const std::function<double(const Apple &, const Apple &)> apple_to_apple;
    };

    const GreatCircleMetric gsm(Eigen::Vector3d(0.0, 0.0, 2.2));

    Metrics metrics[] = {
            { "euclidean" ,
              [&](const Apple &apple) { return (apple.center - start_end_effector_pos).norm(); },
              [&](const Apple &apple_a, const Apple &apple_b) {return (apple_a.center - apple_b.center).norm();}
            },
            { "greatcircle" ,
                        [&](const Apple &apple) { return gsm.measure(apple.center, start_end_effector_pos); },
                        [&](const Apple &apple_a, const Apple &apple_b) { return gsm.measure(apple_a.center, apple_b.center); }
            }
    };

    Json::Value json_values;

    for (Metrics& metric : metrics) {

        Json::Value json_value;
        json_value["metric"] = metric.name;

        auto greedy_order = apple_ordering_from_metric_greedy(
                apples, metric.start_to_apple, metric.apple_to_apple
        );

        json_value["greedy_heuristic_cost"] = ordering_heuristic_cost(greedy_order, apples, metric.start_to_apple, metric.apple_to_apple);

        auto ortools_order = apple_ordering_from_metric_ortools(
                apples, metric.start_to_apple, metric.apple_to_apple
        );

        json_value["ortools_heuristic_cost"] = ordering_heuristic_cost(ortools_order, apples, metric.start_to_apple, metric.apple_to_apple);

        json_values.append(json_value);
    }

    std::ofstream ofs;
    ofs.open("analysis/preorder_experiment_results.json");
    ofs << json_values;
    ofs.close();

//    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
//    auto state_space = std::make_shared<DroneStateSpace>(spec);
//    state_space->setup();
//    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
//    auto planner = std::make_shared<ompl::geometric::AITstar>(si);
//    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
//
//    ompl::base::ScopedState start(si);
//    state_space->copyToOMPLState(start.get(), start_state);
//
//    auto result_path = planOneByOneInOrder(apples, ordering_euclidean, si, planner, objective, start.get());

    return 0;
}


