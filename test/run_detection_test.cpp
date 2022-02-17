#include <gtest/gtest.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>

#include <utility>
#include "../src/experiment_utils.h"
#include "test_utils.h"
#include "../src/multigoal/goals_gnat.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"

TEST(RunDetectionTests, run_detection_test) {

    auto drone = loadRobotModel();
    auto[scene, apples] = createWallApplePlanningScene(drone);

    // Construct the state space for that robot.
    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);
    auto si = initSpaceInformation(scene, drone, state_space);

    auto goals = constructAppleGoals(si, apples);

    ompl::RNG rng;
    std::default_random_engine generator(42);
    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);

    auto gnat = buildGoalGNAT<Apple>(apples, [](const auto &apple) {
        return apple.center;
    });

    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    struct CandidatePath {
        std::set<size_t> goals_visited;
        size_t start_goal, end_goal;

        CandidatePath(std::set<size_t> goalsVisited, const ompl::geometric::PathGeometric &path) : goals_visited(std::move(
                goalsVisited)), path(path) {}

        ompl::geometric::PathGeometric path; // If this uses too much memory, we can also just keep the start/end points.

        [[nodiscard]] double quality() const {
            return (double) std::pow(goals_visited.size(),2) / path.length();
        };

        bool operator<(const CandidatePath &other) const {
            return quality() < other.quality();
        }

    };

    // We'll want a clever way to index this.
    std::set<std::shared_ptr<CandidatePath>> paths;
    std::vector<std::set<std::shared_ptr<CandidatePath>>> paths_ending_at_goals(apples.size());

    // Ideally, we'll adaptively choose these based on demand and/or unexplored regions.
    // For now, we'll just use these paths here as "seeds"
    for (size_t from_goal_id = 0; from_goal_id < apples.size(); ++from_goal_id) {

        std::vector<GNATNode> nearestK;
        gnat.nearestK({from_goal_id, apples[from_goal_id].center}, 5, nearestK);

        for (size_t to_goal_id = 1; to_goal_id < nearestK.size(); to_goal_id++) {

            auto from_sample = std::make_shared<ompl::base::ScopedState<>>(si);

            goals[from_goal_id]->sampleGoal(from_sample->get());

            auto to_sample = std::make_shared<ompl::base::ScopedState<>>(si);
            goals[to_goal_id]->sampleGoal(to_sample->get());

            auto path = ptp.planToOmplState(0.05, from_sample->get(), to_sample->get());

            if (path) {
                std::shared_ptr<CandidatePath> cp(new CandidatePath({from_goal_id, to_goal_id}, *path));
                paths.insert(cp);
                //paths_starting_at_goals[from_goal_id].insert(cp);
                // TODO Do the symmetric case too.
                // Maybe a pass-by case?
            }
        }
    }

    for (size_t goal_id = 0; goal_id < apples.size(); ++goal_id) {
        for (const auto &path : paths_ending_at_goals[goal_id]) {
            for (const auto &path_2 : paths_ending_at_goals[goal_id]) {
                if (path_2.)
                // Could make that a simple collision check, if we're going for ones that are close to each other anyway.
                auto connection = ptp.planToOmplState(0.05, paths[path_a].path.getStates().back(),paths[path_b].path.getStates().back());

//                if (connection) {
//                    CandidatePath path = paths[path_a];
//                    path.path.append(*connection);
//                    path.path.append(paths[path_b].path);
//                    path.goals_visited.insert(paths[path_b].goals_visited.begin(), paths[path_b].goals_visited.end());
//                    paths.insert(path);
//                }
            }
        }
    }
//
//    for (size_t i = 0; i < 100; ++i) {
//
//        auto at = paths.begin();
//        std::advance(at,std::min(std::geometric_distribution<size_t>(0.3)(generator), paths.size()-1));
//
//        // We'll want to index those paths and make this a bit smarter.
//        // If we're lucky, we'll get paths that exponentially double in length and cover all the apples in just a few steps.
//        auto [path_a, path_b] = generateIndexPairNoReplacement(generator, paths.size());
//
//        // Could make that a simple collision check, if we're going for ones that are close to each other anyway.
//        auto connection = ptp.planToOmplState(0.05, paths[path_a].path.getStates().back(),paths[path_b].path.getStates().back());
//
//        if (connection) {
//            CandidatePath path = paths[path_a];
//            path.path.append(*connection);
//            path.path.append(paths[path_b].path);
//            path.goals_visited.insert(paths[path_b].goals_visited.begin(), paths[path_b].goals_visited.end());
//            paths.insert(path);
//        }
//    }

    for (const auto &item : paths) {
        std::cout << "Quality: " << item.quality() << std::endl;
    }


}