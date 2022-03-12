
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <fstream>
#include <ompl/geometric/planners/prm/PRMstar.h>
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
        double state_to_state_distance;
        double state_to_goal_distance;
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
            

            auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);


            double euclidean_distance = (apples[pair.from_target].center - apples[pair.to_target].center).norm();
            double gcm_distance = gsm.measure(apples[pair.from_target].center, apples[pair.to_target].center);

            double state_to_state_distance = INFINITY;

            {
                auto planner = std::make_shared<ompl::geometric::PRMstar>(si);

                auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner->getSpaceInformation());
                pdef->setOptimizationObjective(objective);
                pdef->setStartAndGoalStates(pair.from_state.get(), pair.to_state.get());
                planner->setProblemDefinition(pdef);
                if (planner->solve(ompl::base::timedPlannerTerminationCondition(2.0)) ==
                    ompl::base::PlannerStatus::EXACT_SOLUTION) {
                    state_to_state_distance = pdef->getSolutionPath()->length();
                }
            }

            double state_to_goal_distance = INFINITY;

            {
                // FIXME: Use the right planner when bug 885 in OMPL gets fixed.
                auto planner = std::make_shared<ompl::geometric::PRMstar>(si);

                auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner->getSpaceInformation());
                pdef->setOptimizationObjective(objective);
                pdef->addStartState(pair.from_state.get());
                pdef->setGoal(std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apples[pair.to_target].center));
                planner->setProblemDefinition(pdef);
                if (planner->solve(ompl::base::timedPlannerTerminationCondition(2.0)) ==
                    ompl::base::PlannerStatus::EXACT_SOLUTION) {
                    state_to_goal_distance = pdef->getSolutionPath()->length();
                }
            }

            return {
                    state_to_state_distance,
                    state_to_goal_distance,
                    euclidean_distance,
                    gcm_distance
            };
        }));
    }
    
    Json::Value json;
    for (auto &item : results) {
        Json::Value pair_json;
        const PlanResult &result = item.get();
        pair_json["state_to_state_trajectory_length"] = result.state_to_state_distance;
        pair_json["state_to_goal_trajectory_length"] = result.state_to_goal_distance;
        pair_json["euclidean_distance"] = result.euclidean_distance;
        pair_json["greatcircle_distance"] = result.greatcircle_distance;
        json.append(pair_json);
    }

    std::ofstream ofs;
    ofs.open("analysis/greatcircle_euclidean_actual.json");
    ofs << json;
    ofs.close();

}