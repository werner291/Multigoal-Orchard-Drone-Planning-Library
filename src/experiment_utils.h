
#ifndef NEW_PLANNERS_EXPERIMENT_UTILS_H
#define NEW_PLANNERS_EXPERIMENT_UTILS_H

#include "procedural_tree_generation.h"
#include <moveit/robot_state/conversions.h>
#include <fcl/fcl.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <json/json.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include "multigoal/MetricTwoOpt.h"
#include "SamplerWrapper.h"
#include "LeavesCollisionChecker.h"
#include "ompl_custom.h"
#include "BulletContinuousMotionValidator.h"
#include "InverseClearanceIntegralObjective.h"
#include "planning_scene_diff_message.h"
#include "msgs_utilities.h"
#include "../src/ompl_custom.h"

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

TreePlanningScene buildPlanningScene(int numberOfApples, moveit::core::RobotModelPtr &drone);

moveit::core::RobotModelPtr loadRobotModel();

#endif //NEW_PLANNERS_EXPERIMENT_UTILS_H
