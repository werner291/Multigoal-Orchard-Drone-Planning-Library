
#ifndef NEW_PLANNERS_EXPERIMENT_UTILS_H
#define NEW_PLANNERS_EXPERIMENT_UTILS_H

#include <json/json.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include "../SamplerWrapper.h"
#include "../planning_scene_diff_message.h"
#include "../procedural_tree_generation.h"
#include "../RobotPath.h"
#include "../DronePathLengthObjective.h"
#include "../ompl_custom.h"

struct LeafCollisions {
    double t;
    size_t new_contacts;
    size_t removed_contacts;
};

struct TreePlanningScene {
    std::vector<Apple> apples;
    std::vector<Eigen::Vector3d> leaf_vertices;
    planning_scene::PlanningScenePtr scene;
};

moveit::core::RobotModelPtr loadRobotModel(double base_joint_weight = 1.0);

std::vector<ompl::base::GoalPtr> constructNewAppleGoals(const std::shared_ptr<ompl::base::SpaceInformation> &si, const std::vector<Apple> &apples);

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


struct DroneAndStateSpace {
    moveit::core::RobotModelPtr robot;
    std::shared_ptr<DroneStateSpace> state_space;
};

planning_scene::PlanningScenePtr setupPlanningScene(const moveit_msgs::msg::PlanningScene &planning_scene_message,
                                                    const moveit::core::RobotModelConstPtr &drone);

struct ExperimentPlanningContext {
    std::shared_ptr<DroneStateSpace> state_space;
    ompl::base::SpaceInformationPtr si;
    ompl::base::OptimizationObjectivePtr objective;
};

std::vector<Apple> apples_from_connected_components(shape_msgs::msg::Mesh apples_mesh);

struct PointToPointPair {
    size_t from_target;
    std::shared_ptr<ompl::base::State> from_state;

    size_t to_target;
    std::shared_ptr<ompl::base::State> to_state;
};

std::optional<ompl::geometric::PathGeometric>
planFromStateToState(ompl::base::Planner &planner, const ompl::base::OptimizationObjectivePtr &objective,
                     const ompl::base::State *a, const ompl::base::State *b, double duration);

std::optional<ompl::geometric::PathGeometric>
planExactForPdef(ompl::base::Planner &planner, double duration, bool simplify,
                 const std::shared_ptr<ompl::base::ProblemDefinition> &pdef);

std::optional<ompl::geometric::PathGeometric>
planToGoal(ompl::base::Planner &planner,
           const ompl::base::OptimizationObjectivePtr &objective,
           const ompl::base::State *a,
           double duration,
           bool simplify,
           const ompl::base::GoalPtr &goal);

struct MultiApplePlanResult {

    struct AppleVisit {
        size_t apple_id;
        size_t path_state_index;
    };

    RobotPath path;
    std::vector<AppleVisit> apple_visits;

};

/**
 * @brief Generates a random state for a drone model that lies outside of the tree.
 *
 * @param drone A shared pointer to the constant robot model for the drone.
 * @param seed The seed for the random number generator.
 * @return The generated random state for the drone.
 */
moveit::core::RobotState randomStateOutsideTree(const moveit::core::RobotModelConstPtr &drone, const int seed);

/**
 * @brief Load a Moveit-based state space with the drone.
 *
 * @warning This function nor the returned value are not thread-safe, even if const. The cause is unknown.
 *
 * @param model The Moveit robot model.
 *
 * @return A shared pointer to the loaded state space.
 */
std::shared_ptr<DroneStateSpace> omplStateSpaceForDrone(const moveit::core::RobotModelConstPtr &model);

/**
 * @brief Load the space information that represents the given scene, including all the weird idiosyncracies of how we do planning.
 *
 * @warning This function is not thread-safe.
 *
 * @param stateSpace The state space to be loaded.
 * @param scene_info The information about the planning scene.
 *
 * @return A shared pointer to the loaded space information.
 */
ompl::base::SpaceInformationPtr
loadSpaceInformation(const std::shared_ptr<DroneStateSpace> &stateSpace, const AppleTreePlanningScene &scene_info);

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh);

#endif //NEW_PLANNERS_EXPERIMENT_UTILS_H
