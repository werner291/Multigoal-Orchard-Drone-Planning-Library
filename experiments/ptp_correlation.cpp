
#include <cstddef>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../src/experiment_utils.h"
#include "../src/json_utils.h"
#include <fstream>
#include <filesystem>

/// Data about a single point-to-point planning result.
struct PointToPointPlanResultStat {
    size_t scene_id{};
    size_t pair_id{};
    double euclidean_distance{};
    std::optional<double> actual_distance;
    double planning_time_seconds{};
    bool cleared;
};

Json::Value toJson(const PointToPointPlanResultStat &ptp_stats) {
    Json::Value ptp;
    ptp["scene_id"] = (int) ptp_stats.scene_id;
    ptp["pair_id"] = (int) ptp_stats.pair_id;
    ptp["euclidean_distance"] = ptp_stats.euclidean_distance;
    ptp["planning_time"] = ptp_stats.planning_time_seconds;
    ptp["cleared"] = ptp_stats.cleared;
    if (ptp_stats.actual_distance) { ptp["actual_distance"] = *ptp_stats.actual_distance; }
    else { ptp["actual_distance"] = Json::nullValue; }
    return ptp;
}

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    const Json::Value trees_data = jsonFromGzipFile("test_robots/trees/trees_2021-11-20T12:21:28Z.json.gzip");

    const size_t MAX_TREES = 50;

    Json::Value stats;

    const std::string PATH = "analysis/ptp_stats.json";

    {
        std::ifstream statfile(PATH);
        if (statfile.is_open()) {
            statfile >> stats;
            statfile.close();
        }
    }

    auto last_access = std::chrono::steady_clock::now();

    for (bool clearBetweenRuns: {false, true}) {
        for (double time: {0.01, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0}) {
            for (size_t scene_id = 0; scene_id < trees_data.size() && scene_id < MAX_TREES; ++scene_id) {

                Json::Value &stats_entry = stats[clearBetweenRuns ? "cleared" : "maintained"][std::to_string(
                        time)][(Json::ArrayIndex) scene_id];

                const size_t PTP_PER_TREE = 10;

                if (stats_entry.isArray() && stats_entry.size() == PTP_PER_TREE) {
                    std::cout << "Result is cached." << std::endl;
                } else {

                    std::cout << "Tree: " << scene_id << std::endl;

                    auto tree_data = *treeSceneFromJson(trees_data[(Json::ArrayIndex) scene_id]);

                    auto planning_scene = constructPlanningScene(tree_data, drone);
                    auto si = initSpaceInformation(planning_scene, drone, state_space);
                    auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
                    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
                    auto prms = std::make_shared<ompl::geometric::PRMstar>(si);

                    auto goals = constructAppleGoals(si, tree_data.apples);

                    auto destroyState = [&](ompl::base::State *st) {
                        state_space->freeState(st);
                    };

                    // Similar to ScopedState, but I trust this implementation more with regards to copying and such.
                    typedef std::unique_ptr<ompl::base::State, decltype(destroyState)> StateUptr;

                    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

                    for (size_t ptp_t = 0; ptp_t < PTP_PER_TREE; ++ptp_t) {

                        std::mt19937 gen(ptp_t); // Fixed seed is the ptp index.

                        auto index_pair = generateIndexPairNoReplacement(gen, goals.size());

                        if (clearBetweenRuns) ptp.getPlanner()->clear();

                        StateUptr from_state(state_space->allocState(), destroyState);
                        goals[index_pair.first]->sampleGoal(from_state.get());

                        double euclidean_distance = (tree_data.apples[index_pair.first].center -
                                                     tree_data.apples[index_pair.second].center).norm();

                        if (si->isValid(from_state.get())) {

                            PointToPointPlanResultStat ptp_stat;

                            ptp_stat.euclidean_distance = euclidean_distance;
                            ptp_stat.planning_time_seconds = time;
                            ptp_stat.scene_id = scene_id;
                            ptp_stat.pair_id = ptp_t;
                            ptp_stat.cleared = clearBetweenRuns;

                            auto plan_result = ptp.planToOmplGoal(time, from_state.get(), goals[index_pair.second]);

                            if (plan_result) {
                                ptp_stat.actual_distance = plan_result->length();
                            } else {
                                std::cout << "Planning failed" << std::endl;
                            }

                            stats_entry[(Json::ArrayIndex) ptp_t] = toJson(ptp_stat);
                        }
                    }
                }

                if (std::chrono::steady_clock::now() - last_access > std::chrono::seconds(60)) {
                    last_access = std::chrono::steady_clock::now();
                    std::cout << "Saving results so far." << std::endl;
                    std::ofstream statfile_out(PATH);
                    statfile_out << stats;
                    statfile_out.close();
                }
            }
        }
    }

    {
        std::ofstream statfile_out(PATH);
        statfile_out << stats;
        statfile_out.close();
    }

    return 0;
}

