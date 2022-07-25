
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <vector>
#include <Eigen/Geometry>
#include <json/json.h>
#include <fstream>
#include "../procedural_tree_generation.h"
#include "../src/preorder_strategy/preorder_strategy.h"
#include "../experiment_utils.h"

int main(int argc, char **argv) {

    Json::Value output_json;

    for (size_t scene_idx = 0; scene_idx < 10; ++scene_idx) {

        std::cout << "Scene ID: " << scene_idx << std::endl;

        Json::Value scene_results;

        std::vector<DetachedTreeNode> treeFlattened = make_tree_branches(Eigen::Isometry3d::Identity(), 10, 0.5);
        std::vector<Apple> apples_in_tree = spawn_apples(treeFlattened, 50, 0.05);
        std::vector<Eigen::Vector3d> apples;
        for (const auto &item: apples_in_tree) apples.push_back(item.center);

        Eigen::Vector3d start_point(5.0, 5.0, 5.0);

        std::vector<PreorderStrategy::Solution> solutions;

        KClosestNearestNeighbourOrder().generate_proposals(start_point, apples,
                                                           [&](const std::vector<size_t> &solution) {
                                                               scene_results["kstart-nn"].append(
                                                                       PreorderStrategy::Solution{solution}.length(
                                                                               start_point, apples));
                                                               return false;
                                                           });

        BranchAndBoundOptimal().generate_proposals(start_point, apples,
                                                   [&](const std::vector<size_t> &solution) {
                                                       scene_results["b&b"].append(
                                                               PreorderStrategy::Solution{solution}.length(
                                                                       start_point, apples));
                                                       return false;
                                                   });

        for (double initial_temperature: {0.0, 0.00001, 0.0001}) {
            for (double cooldown_rate: {0.99990, 0.99995, 0.99999}) {
                // Simulated annealing.
                size_t iterations = 5000;

                std::stringstream namess;
                namess << "SN" << initial_temperature << ":" << cooldown_rate << std::endl;
                std::string name = namess.str();

                SimulatedAnnealingBySwapping(cooldown_rate, initial_temperature).generate_proposals(
                        start_point, apples,
                        [&](const std::vector<size_t> &solution) {
                            scene_results[name].append(
                                    PreorderStrategy::Solution{solution}.length(start_point, apples));
                            if (iterations % 1000 == 0) {
                                std::cout << "Iterations: " << iterations << std::endl;
                            }
                            return (--iterations > 0);
                        }
                );
            }
        }


        output_json.append(scene_results);
    }

    std::ofstream outfile("analysis/euclidean_data.json");
    outfile << output_json;
    outfile.close();

}