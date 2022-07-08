
#include <utility>
#include <fstream>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <range/v3/all.hpp>
#include "../src/SingleGoalPlannerMethods.h"
#include "../src/NewKnnPlanner.h"
#include "../src/probe_retreat_move.h"
#include "../src/greatcircle.h"
#include "../src/experiment_utils.h"
#include <boost/range/adaptor/transformed.hpp>
#include <execution>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/Planner.h>
#include <boost/range/combine.hpp>
#include <cstddef>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <geometric_shapes/shape_operations.h>
#include <ompl/geometric/PathSimplifier.h>

#include "../src/planning_scene_diff_message.h"
#include "../src/msgs_utilities.h"
#include "experiment_utils.h"
#include "json_utils.h"
#include "general_utilities.h"
#include "DroneStateConstraintSampler.h"


TreePlanningScene buildPlanningScene(int numberOfApples, moveit::core::RobotModelPtr &drone) {

    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);

    std::vector<DetachedTreeNode> treeFlattened = make_tree_branches(Eigen::Isometry3d::Identity(), 10, 0.5);
    std::vector<Eigen::Vector3d> leafVertices = generateLeafVertices(treeFlattened);
    const double appleRadius = 0.05;
    std::vector<Apple> apples = spawn_apples(treeFlattened, numberOfApples, appleRadius);

    scene->setPlanningSceneDiffMsg(createPlanningSceneDiff(treeFlattened, leafVertices, appleRadius, apples));

    // Diff message apparently can't handle partial ACM updates?
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    return {apples, leafVertices, scene};
}

std::vector<LeafCollisions> collectLeafCollisionStats(const LeavesCollisionChecker &leavesCollisionChecker,
                                                      const robot_trajectory::RobotTrajectory &trajectory) {

    assert(!trajectory.empty());

    Json::Value leaf_collision_stats;

    auto scratchState = std::make_shared<moveit::core::RobotState>(trajectory.getWayPoint(0).getRobotModel());

    std::vector<LeafCollisions> stats;

    std::set<size_t> leaves;
    for (size_t ti = 0; ti < 10000; ti++) {

        double t = (double) ti * trajectory.getDuration() / 10000.0;

        trajectory.getStateAtDurationFromStart(t, scratchState);

        std::set<size_t> new_leaves = leavesCollisionChecker.checkLeafCollisions(*scratchState);

        std::set<size_t> added_leaves;
        std::set_difference(new_leaves.begin(), new_leaves.end(), leaves.begin(), leaves.end(),
                            std::inserter(added_leaves, added_leaves.end()));

        std::set<size_t> removed_leaves;
        std::set_difference(leaves.begin(), leaves.end(), new_leaves.begin(), new_leaves.end(),
                            std::inserter(removed_leaves, removed_leaves.end()));

        if (!added_leaves.empty() || !removed_leaves.empty()) {

            stats.push_back(LeafCollisions{
                    t, added_leaves.size(), removed_leaves.size()
            });
        }

        leaves = new_leaves;

    }

    return stats;
}

moveit::core::RobotModelPtr loadRobotModel() {

    auto urdf = std::make_shared<urdf::Model>();
    urdf->initFile("test_robots/urdf/bot.urdf");

    auto srdf = std::make_shared<srdf::Model>();
    srdf->initFile(*urdf, "test_robots/config/aerial_manipulator_drone.srdf");

    auto robot = std::make_shared<moveit::core::RobotModel>(urdf, srdf);

    // By default, the joint distance factor gets set to the dimension count of the joint, see:
    //     https://github.com/ros-planning/moveit/blob/fd36674cc327962aaf27925ddf1ba9c6a8667d35/moveit_core/robot_model/src/robot_model.cpp#L969
    // I have no idea why this is, but I don't like it.
    for (auto &item: robot->getActiveJointModels()) item->setDistanceFactor(1.0);

    return robot;
}

std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>>
constructAppleGoals(const std::shared_ptr<ompl::base::SpaceInformation> &si, const std::vector<Apple> &apples) {
    static const double GOAL_END_EFFECTOR_RADIUS = 0.01;

    std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> goals;
    for (const auto &apple: apples)
        goals.push_back(
                std::make_shared<DroneEndEffectorNearTarget>(si, GOAL_END_EFFECTOR_RADIUS, apple.center));
    return goals;
}

planning_scene::PlanningScenePtr
constructPlanningScene(const TreeSceneData &tsd, const moveit::core::RobotModelConstPtr &drone) {

    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);

    scene->setPlanningSceneDiffMsg(createPlanningSceneDiff(tsd.branches, tsd.leaf_vertices, 0.05, tsd.apples));

    // Diff message apparently can't handle partial ACM updates?
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    return scene;
}

std::vector<std::vector<PtpSpec>> genPointToPointSpecs(const moveit::core::RobotModelPtr &drone,
                                                       const Json::Value &trees_data, std::mt19937 &gen,
                                                       size_t pairsPerTree) {

    std::vector<std::vector<PtpSpec>> ptp_specs;

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");

    for (const auto &tree: trees_data) {

        ptp_specs.emplace_back(/* empty vector */);

        auto state_space = std::make_shared<DroneStateSpace>(spec);
        // Decode the JSON object into tree data.
        auto tree_data = *treeSceneFromJson(tree);

        // Build the space information, planning scene, etc...
        auto planning_scene = constructPlanningScene(tree_data, drone);
        auto si = initSpaceInformation(planning_scene, drone, state_space);

        for (size_t ptp_t = 0; ptp_t < pairsPerTree; ++ptp_t) {

            // Pick a pair of goals to plan between.
            auto index_pair = generateIndexPairNoReplacement(gen, tree["apples"].size());

            ompl::base::ScopedState from_state(si);
            DroneEndEffectorNearTarget(si, 0.01, fromJsonVector3d(
                    tree["apples"][(Json::ArrayIndex) index_pair.first]["center"])).sampleGoal(from_state.get());

            moveit::core::RobotState start_state(drone);
            state_space->copyToRobotState(start_state, from_state.get());
            start_state.update(true);

            ptp_specs.back().push_back({start_state, index_pair.first, index_pair.second});

        }
    }

    return ptp_specs;
}

planning_scene::PlanningScenePtr
setupPlanningScene(const moveit_msgs::msg::PlanningScene &planning_scene_message,
                   const moveit::core::RobotModelConstPtr &drone) {
    auto scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setPlanningSceneDiffMsg(planning_scene_message);
    // Diff message apparently can't handle partial ACM updates?
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    return scene;
}

std::vector<Apple> apples_from_connected_components(shape_msgs::msg::Mesh apples_mesh) {
    auto connected_components = connected_vertex_components(apples_mesh);

    return boost::copy_range<std::vector<Apple>>(
            connected_components |
            boost::adaptors::transformed([&](const auto &members) {
                Eigen::AlignedBox3d bb;
                for (const auto &vertex_id : members) {
                    bb.extend(Eigen::Vector3d(apples_mesh.vertices[vertex_id].x,apples_mesh.vertices[vertex_id].y,apples_mesh.vertices[vertex_id].z));
                }
                return Apple {bb.center(),Eigen::Vector3d(0.0,0.0,0.0)};
            })
    );
}


std::vector<PointToPointPair>
samplePlanningPairs(const planning_scene::PlanningSceneConstPtr &scene,
                    const moveit::core::RobotModelConstPtr &drone,
                    const std::vector<Apple> &apples, const size_t num_samples) {

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);
    state_space->setup();

    std::default_random_engine rng;

    auto si = initSpaceInformation(scene, drone, state_space);
    auto goals = constructAppleGoals(si, apples);

    auto del_state = [state_space = state_space](ompl::base::State *st) { state_space->freeState(st); };

    return boost::copy_range<std::vector<PointToPointPair>>(
            boost::irange<size_t>(0, num_samples) |
            boost::adaptors::transformed(
                    [apples = apples, state_space = state_space, &rng, &del_state, &goals, &si](size_t i) {

                        auto[target_i, target_j] = generateIndexPairNoReplacement(rng, apples.size());

                        std::shared_ptr<ompl::base::State> from_state(state_space->allocState(), del_state);
                        goals[target_i]->sampleGoal(from_state.get());

                        std::shared_ptr<ompl::base::State> to_state(state_space->allocState(), del_state);
                        goals[target_j]->sampleGoal(to_state.get());

                        return PointToPointPair{target_i, from_state, target_j, to_state};

                    }));
}

std::optional<ompl::geometric::PathGeometric>
planExactForPdef(ompl::base::Planner &planner,
                 double duration,
                 bool simplify,
                 const std::shared_ptr<ompl::base::ProblemDefinition> &pdef) {

    if (planner.solve(ompl::base::timedPlannerTerminationCondition(duration)) == ompl::base::PlannerStatus::EXACT_SOLUTION) {

        ompl::geometric::PathGeometric path = *pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

        if (simplify) {
            ompl::geometric::PathSimplifier(planner.getSpaceInformation()).simplifyMax(path);
        }

        return {path};

    } else {
        return {};
    }
}

std::optional<ompl::geometric::PathGeometric>
planToGoal(ompl::base::Planner &planner,
           const ompl::base::OptimizationObjectivePtr &objective,
           const ompl::base::State *a,
           double duration,
           bool simplify,
           const ompl::base::GoalPtr &goal) {
    
    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner.getSpaceInformation());
    pdef->setOptimizationObjective(objective);
    pdef->addStartState(a);
    pdef->setGoal(goal);

    planner.setProblemDefinition(pdef);

    return planExactForPdef(planner, duration, simplify, pdef);
}

std::optional<ompl::geometric::PathGeometric>
planFromStateToApple(ompl::base::Planner &planner, const ompl::base::OptimizationObjectivePtr &objective,
                     ompl::base::State *a, const Apple &b, double duration, bool simplify) {

    return planToGoal(planner, objective, a, duration, simplify,
                      std::make_shared<DroneEndEffectorNearTarget>(planner.getSpaceInformation(), 0.05, b.center));

}

[[deprecated("use randomStateOutsideTree instead")]]
moveit::core::RobotState stateOutsideTree(const moveit::core::RobotModelConstPtr &drone) {
    moveit::core::RobotState start_state(drone);

    start_state.setVariablePositions({
                                             3.0, 3.0, 1.5,      // Position off the side of the tree
                                             0.0, 0.0, 0.0, 1.0, // Identity rotation
                                             0.0, 0.0, 0.0, 0.0  // Arm straight out
                                     });

    start_state.update(true);
    return start_state;
}

ExperimentPlanningContext loadContext(const moveit::core::RobotModelConstPtr &drone, const moveit_msgs::msg::PlanningScene &scene_msg) {
    ExperimentPlanningContext context;

    // initialize the state space and such
    auto state_space = std::make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), TRANSLATION_BOUND);

    auto scene = setupPlanningScene(scene_msg, drone);
    auto si = initSpaceInformation(scene, scene->getRobotModel(), state_space);

    auto objective = std::make_shared<DronePathLengthObjective>(si);

    return {
            state_space,
            si,
            objective
    };

}

moveit::core::RobotState randomStateOutsideTree(const moveit::core::RobotModelPtr &drone) {
    moveit::core::RobotState start_state(drone);

    randomizeUprightWithBase(start_state, 0.0);

    ompl::RNG rng;

    double random_t = rng.uniformReal(-M_PI, M_PI);
    double height = rng.uniformReal(0.5, 2.0);

    double radius = 4.0;

    start_state.setVariablePosition(0, cos(random_t)*radius);
    start_state.setVariablePosition(1, sin(random_t)*radius);
    start_state.setVariablePosition(2, height);

    start_state.update(true);

    return start_state;
}

std::vector<ompl::base::GoalPtr> constructNewAppleGoals(
        const std::shared_ptr<ompl::base::SpaceInformation> &si,
        const std::vector<Apple> &apples) {

    return apples | ranges::views::transform([&,si=si](auto&& apple) {
                        auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center);
                        return std::static_pointer_cast<ompl::base::Goal>(goal);
                    }) | ranges::to_vector;
}