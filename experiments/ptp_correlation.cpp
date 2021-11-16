
#include <cstddef>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../src/experiment_utils.h"
#include <fstream>

/// Data about a single point-to-point planning result.
struct PointToPointPlanResultStat {
    size_t scene_id{};
    size_t pair_id{};
    double euclidean_distance{};
    std::optional<double> actual_distance;
    double planning_time_seconds{};
};

Json::Value toJson(const PointToPointPlanResultStat &ptp_stats) {
    Json::Value ptp;
    ptp["scene_id"] = (int) ptp_stats.scene_id;
    ptp["pair_id"] = (int) ptp_stats.pair_id;
    ptp["euclidean_distance"] = ptp_stats.euclidean_distance;
    ptp["planning_time"] = ptp_stats.planning_time_seconds;
    if (ptp_stats.actual_distance) { ptp["actual_distance"] = *ptp_stats.actual_distance; }
    else { ptp["actual_distance"] = Json::nullValue; }
    return ptp;
}

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    std::vector<std::vector<PointToPointPlanResultStat>> stats(100);

    for (size_t scene_id = 0; scene_id < stats.size(); ++scene_id) {
        std::cout << "Tree: " << scene_id << std::endl;

        auto tree_scene = buildPlanningScene(50 /* TODO vary? */, drone);
        auto si = initSpaceInformation(tree_scene.scene, drone, state_space);
        auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
        auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
        auto prms = std::make_shared<ompl::geometric::PRMstar>(si);

        std::random_device rd;
        std::mt19937 gen(rd());

        auto goals = constructAppleGoals(tree_scene, si);

        auto destroyState = [&](ompl::base::State *st) {
            state_space->freeState(st);
        };

        // Similar to ScopedState, but I trust this implementation more with regards to copying and such.
        typedef std::unique_ptr<ompl::base::State, decltype(destroyState)> StateUptr;

        struct StateGoalPair {
            size_t scene_id;
            size_t pair_id;
            double euclidean_distance;
            StateUptr state;
            std::shared_ptr<ompl::base::GoalSampleableRegion> goal;
        };

        std::vector<StateGoalPair> pairs;

        for (size_t ptp_t = 0; ptp_t < 10; ++ptp_t) {

            auto index_pair = generateIndexPairNoReplacement(gen, goals.size());

            StateUptr from_state(state_space->allocState(), destroyState);
            goals[index_pair.first]->sampleGoal(from_state.get());

            pairs.push_back({
                                    scene_id, ptp_t,
                                    (tree_scene.apples[index_pair.first].center -
                                     tree_scene.apples[index_pair.second].center).norm(),
                                    std::move(from_state),
                                    goals[index_pair.second]
                            });
        }

        for (double time: {0.01, 0.05, 0.1, 0.2, 0.5, 1.0}) {

            PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

            for (auto &pair: pairs) {

                if (si->isValid(pair.state.get())) {

                    PointToPointPlanResultStat ptp_stat;

                    ptp_stat.euclidean_distance = pair.euclidean_distance;
                    ptp_stat.planning_time_seconds = time;
                    ptp_stat.scene_id = scene_id;
                    ptp_stat.pair_id = pair.pair_id;

                    auto plan_result = ptp.planToOmplGoal(time, pair.state.get(), pair.goal);

                    PointToPointPlanResultStat stat;

                    if (plan_result) {
                        ptp_stat.actual_distance = plan_result->length();
                    } else {
                        std::cout << "Planning failed" << std::endl;
                    }

                    stats[scene_id].push_back(ptp_stat);
                }
            }
        }
    }

    Json::Value all_scenes;
    for (const auto &item: stats) {
        Json::Value all_ptps;
        for (const auto &ptp_stats: item) {
            all_ptps.append(toJson(ptp_stats));
        }
        all_scenes.append(all_ptps);
    }

    std::ofstream results("analysis/ptp_statistics.json");
    results << all_scenes;
    results.close();

    return 0;
}