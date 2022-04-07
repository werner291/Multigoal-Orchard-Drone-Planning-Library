
#ifndef NEW_PLANNERS_EXPERIMENT_UTILS_H
#define NEW_PLANNERS_EXPERIMENT_UTILS_H

#include <json/json.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include "SamplerWrapper.h"
#include "multigoal/MetricTwoOpt.h"
#include "planning_scene_diff_message.h"
#include "procedural_tree_generation.h"

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

moveit::core::RobotState stateOutsideTree(const moveit::core::RobotModelPtr &drone);

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

enum PlannerType {
    PRMSTAR, AITSTAR
};

struct PtpExperiment {
    PlannerType planner_type;
    bool clearBetweenRuns{};
    bool useInformedSampling{};
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

visualization_msgs::MarkerArray markers_for_state(const moveit::core::RobotState& state);

struct DroneAndStateSpace {
    moveit::core::RobotModelPtr robot;
    std::shared_ptr<DroneStateSpace> state_space;
};

planning_scene::PlanningScenePtr setupPlanningScene(const moveit_msgs::PlanningScene &planning_scene_message,
                                                    const moveit::core::RobotModelPtr &drone);

std::vector<Apple> apples_from_connected_components(shape_msgs::Mesh apples_mesh);

struct PointToPointPair {
    size_t from_target;
    std::shared_ptr<ompl::base::State> from_state;

    size_t to_target;
    std::shared_ptr<ompl::base::State> to_state;
};

std::vector<PointToPointPair>
samplePlanningPairs(const planning_scene::PlanningSceneConstPtr &scene,
                    const moveit::core::RobotModelConstPtr &drone,
                    const std::vector<Apple> &apples,
                    const size_t num_samples);

std::optional<ompl::geometric::PathGeometric>
planFromStateToState(ompl::base::Planner &planner, const ompl::base::OptimizationObjectivePtr &objective,
                     ompl::base::State *a, ompl::base::State *b, double duration);

std::optional<ompl::geometric::PathGeometric>
planFromStateToApple(ompl::base::Planner &planner, const ompl::base::OptimizationObjectivePtr &objective,
                     ompl::base::State *a, const Apple &b, double duration, bool simplify);



#endif //NEW_PLANNERS_EXPERIMENT_UTILS_H
