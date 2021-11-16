
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <vector>
#include <Eigen/Geometry>
#include <json/json.h>
#include <fstream>
#include "../src/procedural_tree_generation.h"
#include "../src/preorder_strategy/preorder_strategy.h"
#include "../src/experiment_utils.h"


template<typename T>
std::vector<size_t> indexOf(const std::vector<T> &target_positions) {
    std::vector<size_t> current_solution;
    for (size_t i = 0; i < target_positions.size(); ++i) current_solution.push_back(i);
    return current_solution;
}

class SimulatedAnnealingBySwapping : public PreorderStrategy {

    double cooldown_rate;
    double temperature;

public:

    SimulatedAnnealingBySwapping(double cooldown_rate, double initial_temperature)
            : cooldown_rate(cooldown_rate), temperature(initial_temperature) {}

    Solution
    generate_proposals(const Eigen::Vector3d &start_position, const std::vector<Eigen::Vector3d> &target_positions,
                       std::function<bool(std::vector<size_t>)> callback) override {

        Solution current_solution;
        KClosestNearestNeighbourOrder().generate_proposals(start_position, target_positions,
                                                           [&](std::vector<size_t> solution) {
                                                               current_solution = {solution};
                                                               return false; // accept the first one.
                                                           });
//        Solution current_solution { indexOf(target_positions) };

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<size_t>(0, current_solution.ordering.size())(gen);

        while (callback(current_solution.ordering)) {

            auto pair = generateIndexPairNoReplacement(gen, current_solution.ordering.size());

            Solution new_solution = current_solution;

            std::swap(new_solution.ordering[pair.first], new_solution.ordering[pair.second]);

            if (std::generate_canonical<double, 10>(gen) < temperature ||
                new_solution.length(start_position, target_positions) <
                current_solution.length(start_position, target_positions)) {
                current_solution = std::move(new_solution);
            }

            temperature *= cooldown_rate;
        }

        return current_solution;
    }
};


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