
#ifndef NEW_PLANNERS_EXPERIMENT_UTILS_H
#define NEW_PLANNERS_EXPERIMENT_UTILS_H

#include "procedural_tree_generation.h"
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <json/json.h>
#include <ompl/base/OptimizationObjective.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include "multigoal/MetricTwoOpt.h"
#include "SamplerWrapper.h"
#include "planning_scene_diff_message.h"

struct StateProjection {
    ompl_interface::ModelBasedStateSpace *state_space;

    Eigen::Vector3d operator()(const ompl::base::State *state) const;
};


Eigen::Vector3d goalProjection(const ompl::base::Goal *goal);;

struct LeafCollisions {
    double t;
    size_t new_contacts;
    size_t removed_contacts;
};

robot_state::RobotState genStartState(const moveit::core::RobotModelConstPtr &drone);

struct Experiment {
    std::shared_ptr<MultiGoalPlanner> meta_planner;
    std::shared_ptr<ompl::base::Planner> ptp_planner;
    std::shared_ptr<ompl::base::OptimizationObjective> optimization_objective;
    std::shared_ptr<SamplerWrapper> sampler;
    std::chrono::milliseconds time_budget;
};

struct TreePlanningScene {
    std::vector<Apple> apples;
    std::vector<Eigen::Vector3d> leaf_vertices;
    planning_scene::PlanningScenePtr scene;
};

[[deprecated]]
TreePlanningScene buildPlanningScene(int numberOfApples, moveit::core::RobotModelPtr &drone);

moveit::core::RobotModelPtr loadRobotModel();

std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>>
constructAppleGoals(const std::shared_ptr<ompl::base::SpaceInformation> &si, const std::vector<Apple> &apples);

/**
 *
 * Pick a pair of distinct integers between `0` and `collection_size` (excluded) uniformly at random.
 *
 * @tparam RNG
 * @param gen
 * @param collection_size
 * @return
 */
template<class RNG>
std::pair<size_t, size_t> generateIndexPairNoReplacement(RNG &gen, unsigned long collection_size) {

    size_t i = std::uniform_int_distribution<size_t>(0, collection_size - 1)(gen);
    size_t j = std::uniform_int_distribution<size_t>(0, collection_size - 2)(gen);
    if (j >= i) j += 1;

    return std::make_pair(i, j);
}

planning_scene::PlanningScenePtr
constructPlanningScene(const TreeSceneData &tsd, const moveit::core::RobotModelConstPtr &drone);

struct PtpExperiment {
    bool clearBetweenRuns{};
    double time{};
    size_t scene_id{};
};

struct PtpSpec {
    const moveit::core::RobotState start_state;
    size_t from_goal_idx{};
    size_t goal_idx{};
};


std::vector<std::vector<PtpSpec>>
genPointToPointSpecs(const moveit::core::RobotModelPtr &drone, const Json::Value &trees_data, std::mt19937 &gen,
                     size_t pairsPerTree);

#endif //NEW_PLANNERS_EXPERIMENT_UTILS_H
