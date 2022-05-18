
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <fstream>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../src/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../src/DronePathLengthObjective.h"
#include "../src/greatcircle.h"
#include "../src/probe_retreat_move.h"

int main(int argc, char **argv) {

    auto drone = loadRobotModel();
    auto scene = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    auto planning_pairs = samplePlanningPairs(setupPlanningScene(scene.scene_msg, drone), drone, scene.apples, 1000);

    const GreatCircleMetric gsm(scene.sphere_center);

    thread_pool pool(std::thread::hardware_concurrency());

    struct PlanResult {
        double state_to_state_distance_prm;
        double state_to_state_distance_optimized;
        double euclidean_distance;
        double greatcircle_distance;
    };

    std::vector<std::future<PlanResult>> results;
    for (const auto &pair : planning_pairs) {
        results.push_back(pool.submit([&]() -> PlanResult {
            ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
            auto state_space = std::make_shared<DroneStateSpace>(spec, TRANSLATION_BOUND);
            state_space->setup();
            auto si = initSpaceInformation(setupPlanningScene(scene.scene_msg, drone), drone, state_space);

            auto objective = std::make_shared<DronePathLengthObjective>(si);

            double euclidean_distance = (scene.apples[pair.from_target].center - scene.apples[pair.to_target].center).norm();
            double gcm_distance = gsm.measure(scene.apples[pair.from_target].center, scene.apples[pair.to_target].center);

            double state_to_state_distance_prm = INFINITY;
            double state_to_state_distance_optimized = INFINITY;

            {
                auto planner = std::make_shared<ompl::geometric::PRMstar>(si);

                auto plan_result = planFromStateToState(*planner, objective, pair.from_state.get(), pair.to_state.get(), 10.0);

                if (plan_result) {
                    state_to_state_distance_prm = plan_result->length();
                    state_to_state_distance_optimized = optimize(*plan_result, objective, si).length();
                }
            }

            return {
                    state_to_state_distance_prm,
                    state_to_state_distance_optimized,
                    euclidean_distance,
                    gcm_distance
            };
        }));
    }

    Json::Value json;
    for (auto &item : results) {
        Json::Value pair_json;
        const PlanResult &result = item.get();
        pair_json["state_to_state_distance_prm"] = result.state_to_state_distance_prm;
        pair_json["state_to_state_distance_optimized"] = result.state_to_state_distance_optimized;
        pair_json["euclidean_distance"] = result.euclidean_distance;
        pair_json["greatcircle_distance"] = result.greatcircle_distance;
        json.append(pair_json);

        std::cout << "Done " << json.size() << " out of " << planning_pairs.size() << std::endl;
    }

    std::ofstream ofs;
    ofs.open("analysis/greatcircle_euclidean_actual.json");
    ofs << json;
    ofs.close();

}