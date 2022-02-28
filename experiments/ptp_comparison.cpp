
#include <execution>
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <boost/range/combine.hpp>
#include "../src/thread_pool.hpp"

#include "../src/experiment_utils.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"


int main(int argc, char **argv) {

    const double SOLVE_TIME = 20.0;

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    const std::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &)> planner_allocators[] = {
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::RRTstar>(si); },
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::PRMstar>(si); },
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::AITstar>(si); }
    };

    const std::function<ompl::base::OptimizationObjectivePtr(
            const ompl::base::SpaceInformationPtr &)> objective_allocators[] = {
            [](const auto &si) -> ompl::base::OptimizationObjectivePtr {
                return std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
            },
    };

    const std::chrono::milliseconds planning_times [] = {
            std::chrono::milliseconds(50),
            std::chrono::milliseconds(100),
            std::chrono::milliseconds(200),
            std::chrono::milliseconds(500),
            std::chrono::milliseconds(1000),
            std::chrono::milliseconds(2000),
            std::chrono::milliseconds(5000),
            std::chrono::milliseconds(10000),
    };

    auto planning_pairs = samplePlanningPairs(setupPlanningScene(scene_msg, drone), drone, apples, 50);

    struct PlanAttempt {
        std::string planner_name;
        std::string objective_name;
        std::chrono::milliseconds planning_time;
        double result_length;
    };

    thread_pool pool(std::thread::hardware_concurrency());

    auto run_for_pair_fn = [&, scene_msg=scene_msg](const PointToPointPair &pair) {

        std::vector<PlanAttempt> lengths;

        for (const auto &planner_allocator: planner_allocators) {
            for (const auto &objective_allocator: objective_allocators) {
                for (const auto& time: planning_times) {
                    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
                    auto state_space = std::make_shared<DroneStateSpace>(spec);
                    state_space->setup();
                    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);

                    auto planner = planner_allocator(si);
                    const ompl::base::OptimizationObjectivePtr &objective = objective_allocator(si);

                    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
                    pdef->setOptimizationObjective(objective);
                    pdef->setStartAndGoalStates(pair.from_state.get(), pair.to_state.get());

                    planner->setProblemDefinition(pdef);

                    PlanAttempt attempt;
                    attempt.planner_name = planner->getName();
                    attempt.objective_name = objective->getDescription();
                    attempt.planning_time = time;

                    auto planning_start = std::chrono::steady_clock::now();

                    if (planner->solve((double) time.count() / 1000.0) == ompl::base::PlannerStatus::EXACT_SOLUTION) {
                        attempt.result_length = pdef->getSolutionPath()->length();
                    } else {
                        attempt.result_length = INFINITY;
                    }

                    lengths.push_back(attempt);
                }
            }
        }

        return lengths;
    };

    std::vector<std::future<std::vector<PlanAttempt>>> path_lengths;
    for (const auto &item : planning_pairs) {
        path_lengths.push_back(pool.submit(run_for_pair_fn, item));
    }

    Json::Value value;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ArgumentSelectionDefects" // Seems to trigger a false positive
    for (const auto &[pair, lengths]: boost::combine(planning_pairs, path_lengths)) {
#pragma clang diagnostic pop
        Json::Value run_json;
        run_json["euclidean_distance"] = (apples[pair.from_target].center - apples[pair.to_target].center).norm();
        for (const auto &attempt: boost::get<0>(lengths).get()) {

            Json::Value planner_attempt;
            planner_attempt["time"] = (int) attempt.planning_time.count();
            planner_attempt["length"] = attempt.result_length;
            planner_attempt["objective"] = attempt.objective_name;
            planner_attempt["planner"] = attempt.planner_name;

            run_json["attempts"].append(planner_attempt);

        }
        value.append(run_json);
    }

    std::ofstream ofs;
    ofs.open("analysis/ptp_comparison.json");
    ofs << value;
    ofs.close();

}
