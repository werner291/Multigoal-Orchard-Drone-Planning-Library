
#include <cstddef>
#include "../src/experiment_utils.h"

int main(int argc, char **argv) {


    auto drone = loadRobotModel();

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    struct PointToPointPlanResultStat {
        double euclidean_distance{};
        std::optional<double> actual_distance;
    };

    std::vector<std::vector<PointToPointPlanResultStat>> stats(10);

    for (auto &scene_i: stats) {

        auto tree_scene = buildPlanningScene(50 /* TODO vary? */, drone);
        auto si = initSpaceInformation(tree_scene.scene, drone, state_space);
        auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
        auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
        auto prms = std::make_shared<ompl::geometric::PRMstar>(si);

        std::random_device rd;
        std::mt19937 gen(rd());

        auto goals = constructAppleGoals(tree_scene, si);

        std::vector<PointToPointPlanResultStat> scene_stats(10);

        for (auto &scene_stat: scene_stats) {

            size_t i = std::uniform_int_distribution<size_t>(0, goals.size() - 1)(gen);
            size_t j = std::uniform_int_distribution<size_t>(0, goals.size() - 2)(gen);
            if (j >= i) j += 1;

            ompl::base::ScopedState from(si);
            goals[i]->sampleGoal(from.get());

            ompl::base::ScopedState to(si);
            goals[j]->sampleGoal(to.get());

            assert(si->isValid(from.get()));
            assert(si->isValid(to.get()));

            PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

//            auto plan_result = ptp.planToOmplGoal(1.0, from.get(), goals[j]);
            auto plan_result = ptp.planToOmplState(1.0, from.get(), to.get());

            PointToPointPlanResultStat stat;

            stat.euclidean_distance = (tree_scene.apples[i].center - tree_scene.apples[j].center).norm();
            if (plan_result) {
                stat.actual_distance = plan_result->length();
            } else {
                std::cout << "Planning failed" << std::endl;
            }

            scene_stat = stat;
        }

        scene_i = scene_stats;
    }

    Json::Value all_scenes;
    for (const auto &item: stats) {
        Json::Value all_ptps;
        for (const auto &item2: item) {
            Json::Value ptp;
            ptp["euclidean_distance"] = item2.euclidean_distance;
            if (item2.actual_distance) { ptp["actual_distance"] = *item2.actual_distance; }
            else { ptp["actual_distance"] = Json::nullValue; }
            all_ptps.append(ptp);
        }
        all_scenes.append(all_ptps);
    }

    std::ofstream results("analysis/ptp_statistics.json");
    results << all_scenes;
    results.close();

    return 0;
}