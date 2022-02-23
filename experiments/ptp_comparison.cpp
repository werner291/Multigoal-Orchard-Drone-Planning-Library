
#include <execution>
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <boost/range/combine.hpp>
#include "../src/experiment_utils.h"


int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    ros::init(argc, argv, "planning_test_debug");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    auto scene_topic = nh.advertise<moveit_msgs::PlanningScene>("/debug_scene", 10, true);
    while (scene_topic.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }
    scene_topic.publish(scene_msg);

    auto scene = setupPlanningScene(scene_msg, drone);

    std::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &)> planner_allocators[] = {
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::RRTstar>(si); },
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::PRMstar>(si); },
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::AITstar>(si); }
    };

    const std::string planner_names[] = {"RRTstar","PRMstar","AITstar"};

    auto planning_pairs = samplePlanningPairs(scene, drone, apples, 10);

    {
        ros::init(argc, argv, "planning_test_debug");
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(4);
        spinner.start();

        ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
        auto state_space = std::make_shared<DroneStateSpace>(spec);

        auto scene_topic = nh.advertise<moveit_msgs::PlanningScene>("/debug_scene", 10, true);
        while (scene_topic.getNumSubscribers() == 0) {
            ros::Duration(0.1).sleep();
        }
        scene_topic.publish(scene_msg);

        auto rs_topic = nh.advertise<visualization_msgs::MarkerArray>("/debug_state", 10, true);
        while (rs_topic.getNumSubscribers() == 0) {
            ros::Duration(0.1).sleep();
        }
        while (true) {
            for (const auto &ptp: planning_pairs) {
                moveit::core::RobotState moveit_state(drone);
                state_space->copyToRobotState(moveit_state, ptp.from_state.get());
                moveit_state.update(true);
                rs_topic.publish(markers_for_state(moveit_state));
                ros::Duration(2.0).sleep();
            };
        }
    }

    std::vector<std::vector<double>> path_lengths(planning_pairs.size());

    std::transform(std::execution::par, planning_pairs.begin(), planning_pairs.end(), path_lengths.begin(),
                  [&](const PointToPointPair &pair) {

        std::vector<double> lengths;

        for (const auto &planner_allocator: planner_allocators) {
            ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
            auto state_space = std::make_shared<DroneStateSpace>(spec);
            state_space->setup();
            auto si = initSpaceInformation(scene, drone, state_space);

            auto planner = planner_allocator(si);
            auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
            pdef->setStartAndGoalStates(pair.from_state.get(), pair.to_state.get());

            planner->setProblemDefinition(pdef);

            if (planner->solve(5.0) == ompl::base::PlannerStatus::EXACT_SOLUTION) {
                lengths.push_back(pdef->getSolutionPath()->length());
            } else {
                lengths.push_back(std::numeric_limits<double>::infinity());
            }
        }

        return lengths;
    });

    Json::Value value;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ArgumentSelectionDefects" // Seems to trigger a false positive
    for (const auto &[pair,lengths] : boost::combine(planning_pairs,path_lengths)) {
#pragma clang diagnostic pop
        Json::Value run_json;
        run_json["euclidean_distance"] = (apples[pair.from_target].center - apples[pair.to_target].center).norm();
        for (const auto &[length,planner_name] : boost::combine(boost::get<0>(lengths), planner_names)) {
            run_json["attempts"][boost::get<0>(planner_name)] = (float) length;
        }
    }

    std::ofstream ofs;
    ofs.open("analysis/ptp_comparison.json");
    ofs << value;
    ofs.close();


}
