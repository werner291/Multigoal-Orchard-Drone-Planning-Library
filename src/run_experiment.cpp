
#include "run_experiment.h"
#include "experiment_utils.h"
#include "probe_retreat_move.h"
#include "NewMultiGoalPlanner.h"
#include "DistanceHeuristics.h"
#include "ShellPathPlanner.h"
#include "MultigoalPrmStar.h"
#include "NewKnnPlanner.h"
#include "RoboTSP.h"
#include <range/v3/all.hpp>
#include <fstream>

using namespace std;

std::shared_ptr<DroneStateSpace> loadStateSpace(const moveit::core::RobotModelPtr &model) {

    // initialize the state space and such
    auto stateSpace = make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(model, "whole_body"), TRANSLATION_BOUND);

    return stateSpace;
}

ompl::base::SpaceInformationPtr loadSpaceInformation(
        const std::shared_ptr<DroneStateSpace> &stateSpace,
        const AppleTreePlanningScene &scene_info) {
    auto scene = setupPlanningScene(scene_info.scene_msg, stateSpace->getRobotModel());
    auto si = initSpaceInformation(scene, scene->getRobotModel(), stateSpace);
    return si;
}

Json::Value toJson(const NewMultiGoalPlanner::PlanResult &result) {
    Json::Value run_stats;
    run_stats["final_path_length"] = result.length();
    run_stats["goals_visited"] = (int) result.segments.size();
    return run_stats;
}

ompl::base::ScopedState<> genStartState(const shared_ptr<DroneStateSpace> &stateSpace) {
    auto start_state_moveit = stateOutsideTree(stateSpace->getRobotModel());
    ompl::base::ScopedState<> start_state(stateSpace);
    stateSpace->copyToOMPLState(start_state.get(), start_state_moveit);
    return start_state;
}

void
run_planner_experiment(const std::vector<NewMultiGoalPlannerAllocatorFn> &allocators,
                       const std::string &results_path,
                       const int num_runs,
                       const std::vector<size_t>& napples,
                       const unsigned int nworkers) {

    auto drone = loadRobotModel();
    auto stateSpace = loadStateSpace(drone);

    // Load the apple tree model with some metadata.
    const auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    std::vector<std::thread> threads;
    std::mutex mut;

    Json::Value statistics;

    std::ifstream ifs(results_path);
    if (ifs.is_open()) {
        // Recover from a previous, failed run.
        std::cout << "Loading previous run statistics from " << results_path << std::endl;
        ifs >> statistics;
    }

    // Constant seed so that we get the same batch between runs (in case of crashes)
    auto rng = std::mt19937(42);

    auto start_states =
            ranges::views::cartesian_product(ranges::views::iota(0, num_runs), napples)
            | ranges::views::transform([&](const auto &tuple) {
                const auto run_id = std::get<0>(tuple);
                const auto napples = std::get<1>(tuple);

                // Grab a random subset of the apples of size napples.
                // Type explicitly specified so that we'll get a copy
                std::vector<Apple> apples = scene_info.apples;
                std::shuffle(apples.begin(), apples.end(), rng);
                if (napples < apples.size()) {
                    apples.resize(napples);
                }

                return std::make_tuple(run_id, genStartState(stateSpace), apples);
            });

    auto tasks = ranges::views::cartesian_product(
            allocators,
            start_states
    ) | ranges::to_vector;

    // Shuffle, but use the same seed every time.
    // The purpose of this shuffle is to reduce the effect of the order of the runs.
    // however, we don't really care about true randomness, more about just making sure there's a good variety in the ordering.
    std::shuffle(tasks.begin(), tasks.end(), rng);

    size_t num_tasks = tasks.size();
    size_t current_task = statistics.size();

    const size_t BATCH_SIZE = 20;

    if (!(statistics.size() == num_runs || statistics.size() % BATCH_SIZE == 0)) {
        throw std::runtime_error("Invalid previous run statistics (not a batch size)");
    }

    // Ensure that the task numbers line up to make sure the RNG hasn't been disturbed.
    for (size_t i = 0; i < statistics.size(); i++) {
        if (statistics[(int) i].isMember("start_state") &&
            statistics[(int) i]["start_state"].asInt() != std::get<0>(std::get<1>(tasks[i]))) {
            throw std::runtime_error("Invalid previous run statistics (start state ID mismatch)");
        }
    }

    for (size_t thread_id = 0; thread_id < nworkers; ++thread_id) {
        threads.emplace_back([&]() {

            // Keeping this local.
            auto threadLocalStateSpace = loadStateSpace(drone);

            auto si = loadSpaceInformation(threadLocalStateSpace, scene_info);

            while (true) {

                size_t thread_current_task;

                {
                    std::lock_guard<std::mutex> lock(mut);
                    thread_current_task = current_task++;

                    //the stats already have an entry for this task, so we can skip it.
                    if (statistics[(int)thread_current_task].isMember("start_state")) {
                        std::cout << "Skipping task " << thread_current_task << " because it was already completed." << std::endl;
                        continue;
                    }

                    if (thread_current_task >= num_tasks) {
                        return;
                    }
                }

                std::cout << "Starting task " << thread_current_task << " of " << num_tasks << std::endl;

                const auto &[planner_allocator, start_state_pair] = tasks[thread_current_task];
                const auto &[run_i, start_state, apples] = start_state_pair;

                auto planner = planner_allocator(scene_info, si);
                auto goals = constructNewAppleGoals(si, apples);

                auto objective = make_shared<DronePathLengthObjective>(si);

                auto start_time = ompl::time::now();
                auto result = planner->plan(si, start_state.get(), goals);
                auto run_time = ompl::time::seconds(ompl::time::now() - start_time);

                auto plan_result = toJson(result);
                plan_result["run_time"] = run_time;
                plan_result["start_state"] = (int) run_i;
                plan_result["planner_params"] = planner->parameters();
                plan_result["planner_name"] = planner->name();

                {
                    std::lock_guard<std::mutex> lock(mut);
                    statistics[(int)thread_current_task] = plan_result;
                    std::cout << "Completed run " << thread_current_task << " of " << tasks.size() << std::endl;

                    if (statistics.size() % BATCH_SIZE == 0) {
                        std::cout << "Saving statistics to " << results_path << std::endl;
                        std::ofstream ofs(results_path);
                        ofs << statistics;
                    }
                }
            }
        });
    }

    for (auto &thread: threads) {
        thread.join();
    }

    std::cout << "All runs completed, saving statistics to " << results_path << std::endl;
    std::ofstream ofs;
    ofs.open(results_path);
    ofs << statistics;
    ofs.close();
}

ompl::base::PlannerPtr allocPRM(const ompl::base::SpaceInformationPtr &si) {
    auto planner = make_shared<ompl::geometric::PRM>(si);
    return planner;
}

std::vector<NewMultiGoalPlannerAllocatorFn> make_shellpath_allocators() {

    bool applyShellstateOptimization[] = {true, false};
    bool useImprovisedInformedSampler[] = {false, true};
    bool tryLuckyShots[] = {false, true};
    bool useCostConvergence[] = {false, true};
    double ptp_time_seconds[] = {0.4, 0.5, 1.0};

    ompl::base::PlannerPtr (*planner_allocators[])(const ompl::base::SpaceInformationPtr&) = {&allocPRM};

    return ranges::views::cartesian_product(applyShellstateOptimization,
                                            ptp_time_seconds,
                                            planner_allocators,
                                            useImprovisedInformedSampler,
                                            tryLuckyShots,
                                            useCostConvergence)
           | ranges::views::transform([](const auto tuple) -> NewMultiGoalPlannerAllocatorFn {

        auto [shellOptimize, ptp_budget, allocator, improvised_sampler, tryLucky, costConvergence] = tuple;

        // I... believe this copies them?
        return [shellOptimize=shellOptimize,
                ptp_budget=ptp_budget,
                allocator=allocator,
                improvised_sampler=improvised_sampler,
                tryLucky=tryLucky,
                costConvergence=costConvergence](
                        const AppleTreePlanningScene &scene_info,
                        const ompl::base::SpaceInformationPtr &si) {

            auto shell = std::make_shared<SphereShell>(
                    scene_info.sphere_center,
                    scene_info.sphere_radius
            );

            auto heuristics = std::make_shared<GreatCircleOmplDistanceHeuristics>(
                    GreatCircleMetric(scene_info.sphere_center),
                    std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace())
            );

            auto ptp = std::make_shared<SingleGoalPlannerMethods>(
                    ptp_budget,
                    si,
                    std::make_shared<DronePathLengthObjective>(si),
                    allocator,
                    improvised_sampler,
                    tryLucky,
                    costConvergence
            );

            return std::make_shared<ShellPathPlanner>(shell, shellOptimize, heuristics, ptp);
        };
    }) | ranges::to_vector;
}

std::vector<NewMultiGoalPlannerAllocatorFn> make_tsp_over_prm_allocators() {

    const auto samples_per_goal = ranges::views::iota(2, 10);
    const double plan_times_seconds[] = {1.0, 2.0, 5.0, 10.0, 15.0, 20.0};//, 30.0, 60.0};
    const bool optimize_segments_options[] = {false,true};
    const bool optimize_matrix_options[] = {false};//,true};

    return ranges::views::cartesian_product(plan_times_seconds, samples_per_goal, optimize_segments_options, optimize_matrix_options)
           | ranges::views::filter([](const auto tuple) {
        const auto &[plan_time, samples, optimize_segments, optimize_matrix] = tuple;

        double expected_time = plan_time * (double) (samples * samples);

        if (expected_time >= 600.0) {
            std::cout << "Dropping task with plan_time: " << plan_time << " samples: " << samples << " expected_time: "
                      << expected_time << std::endl;
            return false;
        } else {
            return true;
        }

    }) | ranges::views::transform([&](const auto tuple) -> NewMultiGoalPlannerAllocatorFn {

        auto [plan_time, samples, optimize_segments, optimize_matrix] = tuple;

        return [plan_time=plan_time,
                samples=samples,
                optimize_segments=optimize_segments,
                optimize_matrix=optimize_matrix](
                        const AppleTreePlanningScene &scene_info,
                        const ompl::base::SpaceInformationPtr &si) {
            return std::make_shared<MultigoalPrmStar>(plan_time, samples, optimize_segments,optimize_matrix);
        };
    }) | ranges::to_vector;
}

std::vector<NewMultiGoalPlannerAllocatorFn> make_robo_tsp_allocators() {

    size_t ks[] = {5, 10, 20};
    double ptp_time_seconds[] = {1.0, 2.0, 5.0};

    ompl::base::PlannerPtr (*planner_allocators[])(const ompl::base::SpaceInformationPtr&) = {&allocPRM};

    return ranges::views::cartesian_product(ptp_time_seconds, planner_allocators, ks)
           | ranges::views::transform([](const auto tuple) -> NewMultiGoalPlannerAllocatorFn {

        auto [ptp_budget, allocator, k] = tuple;

        // I... believe this copies them?
        return [ptp_budget=ptp_budget, allocator=allocator, k=k](
                const AppleTreePlanningScene &scene_info,
                const ompl::base::SpaceInformationPtr &si) {

            auto shell = std::make_shared<SphereShell>(
                    scene_info.sphere_center,
                    scene_info.sphere_radius
            );

            auto heuristics = std::make_shared<GreatCircleOmplDistanceHeuristics>(
                    GreatCircleMetric(scene_info.sphere_center),
                    std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace())
            );

            auto ptp = std::make_shared<SingleGoalPlannerMethods>(
                    ptp_budget,
                    si,
                    std::make_shared<DronePathLengthObjective>(si),
                    allocator,
                    true,
                    true,
                    true
            );

            return std::make_shared<RoboTSP>(std::dynamic_pointer_cast<OmplDistanceHeuristics>(heuristics),ptp,k);
        };
    }) | ranges::to_vector;

}

//std::vector<NewMultiGoalPlannerAllocatorFn> make_knn_allocators() {
//
//    std::vector<NewMultiGoalPlannerAllocatorFn> allocators;
//
//    for (size_t k = 1; k <= 3; ++k) {
//
//        allocators.emplace_back([k=k] (const AppleTreePlanningScene& scene_info, const std::shared_ptr<DroneStateSpace>& stateSpace) {
//            const GreatCircleMetric gcm(scene_info.sphere_center);
//            auto heuristic = std::make_shared<GreatCircleOmplDistanceHeuristics>(gcm, stateSpace);
//            auto planner = std::make_shared<NewKnnPlanner>(heuristic, k);
//            return std::static_pointer_cast<NewMultiGoalPlanner>(planner);
//        });
//
//        allocators.emplace_back([k=k] (const AppleTreePlanningScene& scene_info,const std::shared_ptr<DroneStateSpace>& stateSpace) {
//            auto heuristic = std::make_shared<EuclideanOmplDistanceHeuristics>(stateSpace);
//            auto planner = std::make_shared<NewKnnPlanner>(heuristic, k);
//            return std::static_pointer_cast<NewMultiGoalPlanner>(planner);
//        });
//
//    }
//
//    return allocators;
//}
