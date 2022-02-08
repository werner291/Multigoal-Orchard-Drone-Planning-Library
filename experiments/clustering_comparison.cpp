
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include "../src/experiment_utils.h"
#include "../test/test_utils.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/json_utils.h"

int main(int argc, char **argv) {

    // Worth considering: https://www.researchgate.net/publication/220895309_Speeding-Up_Hierarchical_Agglomerative_Clustering_in_Presence_of_Expensive_Metrics

    auto drone = loadRobotModel();

    auto[scene,apples] = createWallApplePlanningScene(drone);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    auto si = initSpaceInformation(scene, drone, state_space);

    static const int SAMPLES_PER_GOAL = 20;

    auto goals = constructAppleGoals(si, apples);

    auto goal_samples = clustering::takeInitialSamples(goals, si, SAMPLES_PER_GOAL);

    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);

    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    clustering::NearestKPreselection preselection;
    clustering::SelectByExponentialRadius postselect_exprad({0.1, 1.5});
    clustering::SelectByMean postselect_mean(1.5);

    clustering::PostSelectionStrategy* post_selections[2] = {&postselect_exprad, &postselect_mean};

    Json::Value clusters_json;

    for (size_t i : boost::irange(0,1)) {
        for (clustering::PostSelectionStrategy *postselect: post_selections) {

            auto clusters = clustering::buildClusters(ptp, goal_samples, preselection, *postselect);

            Json::Value entry;

            entry["postselection"] = postselect->getName();

            std::cout << "Iteration " << i << " post select: " << postselect->getName() << std::endl;

            for (const auto &layer: clusters) {
                Json::Value layer_json;
                for (const auto &cluster: layer) {
                    layer_json.append(toJSON(cluster));
                }
                entry["hierarchy"].append(layer_json);
            }

            clusters_json.append(entry);
        }
    }

    std::ofstream clusters_file;
    clusters_file.open("analysis/clusters.json");
    clusters_file << clusters_json;
    clusters_file.close();

}