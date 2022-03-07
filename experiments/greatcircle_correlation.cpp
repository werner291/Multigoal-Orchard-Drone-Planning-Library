
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <fstream>
#include "../src/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/greatcircle.h"

int main(int argc, char **argv) {

    auto drone = loadRobotModel();
    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    auto planning_pairs = samplePlanningPairs(setupPlanningScene(scene_msg, drone), drone, apples, 500);

    const Eigen::Vector3d sphere_center(0.0,0.0,2.2); // Measured in Blender for the particular tree model we're using.
    double sphere_radius = 1.5;

    const GreatCircleMetric gsm(sphere_center);

    thread_pool pool(std::thread::hardware_concurrency());

    struct PlanResult {
        double result_length;
        double euclidean_distance;
        double greatcircle_distance;
    };

    std::vector<std::future<PlanResult>> results;
    for (const auto &pair : planning_pairs) {
        results.push_back(pool.submit([&, scene_msg=scene_msg, drone=drone,apples=apples]() -> PlanResult {
            ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
            auto state_space = std::make_shared<DroneStateSpace>(spec);
            state_space->setup();
            auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
            auto planner = std::make_shared<ompl::geometric::AITstar>(si);
            auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

            auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner->getSpaceInformation());
            pdef->setOptimizationObjective(objective);
            pdef->setStartAndGoalStates(pair.from_state.get(), pair.to_state.get());

            planner->setProblemDefinition(pdef);

            if (planner->solve(ompl::base::timedPlannerTerminationCondition(1.0)) ==
                ompl::base::PlannerStatus::EXACT_SOLUTION) {
                return {
                        pdef->getSolutionPath()->length(),
                        (apples[pair.from_target].center - apples[pair.to_target].center).norm(),
                        gsm.measure(apples[pair.from_target].center, apples[pair.to_target].center)
                };
            } else {
                return {
                        INFINITY,
                        (apples[pair.from_target].center - apples[pair.to_target].center).norm(),
                        gsm.measure(apples[pair.from_target].center, apples[pair.to_target].center)
                };
            }
        }));
    }
    
    Json::Value json;
    for (auto &item : results) {
        Json::Value pair_json;
        const PlanResult &result = item.get();
        pair_json["result_distance"] = result.result_length;
        pair_json["euclidean_distance"] = result.euclidean_distance;
        pair_json["greatcircle_distance"] = result.greatcircle_distance;
        json.append(pair_json);
    }

    std::ofstream ofs;
    ofs.open("analysis/greatcircle_euclidean_actual.json");
    ofs << json;
    ofs.close();

}