
#include <cstddef>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../src/experiment_utils.h"
#include "../src/json_utils.h"
#include <fstream>
#include <filesystem>
#include <condition_variable>
#include <random>

const std::string PATH = "analysis/ptp_stats.json";

/// Data about a single point-to-point planning result.
struct PointToPointPlanResultStat {
    size_t scene_id{};
    size_t pair_id{};
    double euclidean_distance{};
    std::optional<double> actual_distance;
    double planning_time_seconds{};
    bool cleared;
};


const size_t PTP_PER_TREE = 10;


std::deque<PtpExperiment> makeExperiments(const Json::Value &trees_data, const size_t MAX_TREES);

Json::Value toJson(const PointToPointPlanResultStat &ptp_stats);

Json::Value &getExperimentStats(Json::Value &stats, const PtpExperiment &current) {
    return stats[current.clearBetweenRuns ? "cleared" : "maintained"]
    [std::to_string(current.time)]
    [(Json::ArrayIndex) current.scene_id];
}


int main(int argc, char **argv) {

    // Get the drone model from URDF and SRDF
    auto drone = loadRobotModel();

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");

    // Load the scene data
    const Json::Value trees_data = jsonFromGzipFile("test_robots/trees/trees_2021-11-20T12:21:28Z.json.gzip");

    // Limit ourselves to a subset of all trees.
    const size_t MAX_TREES = 50;

    // Load the cached statistics. (Delete the file to re-run cleanly)
    Json::Value stats = loadJsonFromFile(PATH);

    std::vector<std::vector<PtpSpec>> ptp_specs;

    std::random_device dev;
    std::mt19937 gen(dev());
    if (stats["ptp"].isArray()) {
        std::cout << "Will load planning start/goal pairs from JSON." << std::endl;
        assert(stats["ptp"].size() == trees_data.size());
        ptp_specs = ptpSpecsFromJson(drone, stats["ptp"]);
    } else {
        std::cout << "Will generate start/goal pairs." << std::endl;
        ptp_specs = genPointToPointSpecs(drone, trees_data, gen, PTP_PER_TREE);
        stats["ptp"] = ptpSpecsToJson(ptp_specs);
    }

    // Create a queue of parameter settings to try out.
    std::deque<PtpExperiment> experiments = makeExperiments(trees_data, MAX_TREES);

    // Shuffle to lessen the chance of system load skewing results.
    std::shuffle(experiments.begin(), experiments.end(), std::mt19937(std::random_device()()));

    // The mutex to be locked when accessing the experiment queue and statistics object.
    std::mutex mtx;

    // A condition variable to notify the main thread when a result has been obtained.
    std::condition_variable cv;

    // Init a worker pool.
    std::vector<std::thread> pool;
    for (int i = 0; i < 8; i++) {
        // Create a worker thread.
        pool.emplace_back([&]() {

            // Create a state space for the drone.
            auto state_space = std::make_shared<DroneStateSpace>(spec);

            // Run until the queue is empty, but we can't check that here since we need a lock.
            while (true) {

                // First, we attempt to get the next parameter settings to try.
                PtpExperiment current{};

                // Push a block to ensure the lock guard isn't held too long.
                {
                    std::lock_guard lock(mtx); // Lock since we're accessing the queue.

                    if (experiments.empty()) {
                        return; // The worker terminates if there's no more work.
                    } else {
                        current = experiments.back(); // Grab the next experiment parameters.
                        experiments.pop_back();

                        // Check the cache to see if the experiment has already been performed.
                        const Json::Value &stats_entry = getExperimentStats(stats, current);

                        // A correct entry is an array with PTP_PER_TREE entries.
                        // Fewer entries indicates that the experiment was terminated halfway through.
                        if (stats_entry.isArray() && stats_entry.size() == PTP_PER_TREE) {
                            std::cout << "Result is cached." << std::endl;
                            continue; // Skip this loop iteration, and try to get a new experiment.
                        }
                    }
                } // Pop the block, release the lock.

                std::cout << "Tree: " << current.scene_id << std::endl;

                // Decode the JSON object into tree data.
                auto tree_data = *treeSceneFromJson(trees_data[(Json::ArrayIndex) current.scene_id]);

                // Build the space information, planning scene, etc...
                auto planning_scene = constructPlanningScene(tree_data, drone);
                auto si = initSpaceInformation(planning_scene, drone, state_space);
                auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
                auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
                auto prms = std::make_shared<ompl::geometric::PRMstar>(si);

                // Convert the apples into sampleable goal regons.
                auto goals = constructAppleGoals(si, tree_data.apples);

                // Destructor for State* pointers that uses the state space.
                auto destroyState = [&](ompl::base::State *st) {
                    state_space->freeState(st);
                };

                // Similar to ScopedState, but I trust this implementation more in regard to move semantics
                typedef std::unique_ptr<ompl::base::State, decltype(destroyState)> StateUptr;

                // Initialize the point-to-point planner.
                PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

                for (size_t ptp_t = 0; ptp_t < ptp_specs[current.scene_id].size(); ++ptp_t) {

                    if (current.clearBetweenRuns) ptp.getPlanner()->clear();

                    double euclidean_distance = (
                            tree_data.apples[ptp_specs[current.scene_id][ptp_t].from_goal_idx].center -
                            tree_data.apples[ptp_specs[current.scene_id][ptp_t].goal_idx].center).norm();

                    ompl::base::ScopedState from_state(si);
                    state_space->copyToOMPLState(from_state.get(), ptp_specs[current.scene_id][ptp_t].start_state);

                    PointToPointPlanResultStat ptp_stat;
                    ptp_stat.euclidean_distance = euclidean_distance;
                    ptp_stat.planning_time_seconds = current.time;
                    ptp_stat.scene_id = current.scene_id;
                    ptp_stat.pair_id = ptp_t;
                    ptp_stat.cleared = current.clearBetweenRuns;

                    if (si->isValid(from_state.get())) {
                        auto plan_result = ptp.planToOmplGoal(current.time, from_state.get(),
                                                              goals[ptp_specs[current.scene_id][ptp_t].goal_idx]);

                        if (plan_result) {
                            ptp_stat.actual_distance = plan_result->length();
                        } else {
                            std::cout << "Planning failed" << std::endl;
                        }

                    }

                    {
                        std::lock_guard lock(mtx);
                        getExperimentStats(stats, current)[(Json::ArrayIndex) ptp_t] = toJson(ptp_stat);
                    }
                    cv.notify_one();
                }
            }
        });
    }

    auto last_access = std::chrono::steady_clock::now();

    while (true) {

        std::unique_lock lock(mtx);
        cv.wait(lock);

        if (std::chrono::steady_clock::now() - last_access > std::chrono::seconds(60)) {
            last_access = std::chrono::steady_clock::now();
            std::cout << "Saving results so far." << std::endl;
            std::ofstream statfile_out(PATH);
            statfile_out << stats;
            statfile_out.close();
        }

        if (experiments.empty()) break;
    }

    for (auto &worker: pool) { worker.join(); }

    {
        std::ofstream statfile_out(PATH);
        statfile_out << stats;
        statfile_out.close();
    }

    return 0;
}

std::deque<PtpExperiment> makeExperiments(const Json::Value &trees_data, const size_t MAX_TREES) {
    std::deque<PtpExperiment> experiments;
    for (bool clearBetweenRuns: {false, true}) {
        for (double time: {0.01, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0}) {
            for (size_t scene_id = 0; scene_id < trees_data.size() && scene_id < MAX_TREES; ++scene_id) {
                experiments.push_back({
                                              clearBetweenRuns, time, scene_id
                                      });
            }
        }
    }
    return experiments;
}

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

