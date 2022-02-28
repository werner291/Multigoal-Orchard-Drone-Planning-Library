
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
#include <eigen_conversions/eigen_msg.h>

#include "../src/planning_scene_diff_message.h"
#include "../src/msgs_utilities.h"
#include "experiment_utils.h"
#include "json_utils.h"
#include "general_utilities.h"

robot_state::RobotState genStartState(const moveit::core::RobotModelConstPtr &drone) {
    robot_state::RobotState start_state(drone);
    start_state.setToDefaultValues();

    std::vector<double> positions {
        -10.0, -10.0, 10.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0
    };

    start_state.setJointGroupPositions(drone->getJointModelGroup("whole_body"), positions);
    start_state.update(true);
    return start_state;
}


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
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    return {apples, leafVertices, scene};
}

Eigen::Vector3d goalProjection(const ompl::base::Goal *goal) {
    return goal->as<DroneEndEffectorNearTarget>()->getTarget();
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

Eigen::Vector3d StateProjection::operator()(const ompl::base::State *state) const {
    moveit::core::RobotState st(state_space->getRobotModel());
    state_space->copyToRobotState(st, state);
    st.update(true);
    return st.getGlobalLinkTransform("end_effector").translation();
}

moveit::core::RobotModelPtr loadRobotModel() {
    auto urdf = std::make_shared<urdf::Model>();
    assert(urdf->initFile("test_robots/urdf/bot.urdf"));

    auto srdf = std::make_shared<srdf::Model>();
    assert(srdf->initFile(*urdf, "test_robots/config/aerial_manipulator_drone.srdf"));

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
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

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

visualization_msgs::MarkerArray markers_for_state(const moveit::core::RobotState& state) {

    auto lms = state.getRobotModel()->getLinkModelsWithCollisionGeometry();

    visualization_msgs::MarkerArray ma;

    for (auto lm : lms) {

        Eigen::Isometry3d xfm = state.getGlobalLinkTransform(lm);

        assert(lm->getCollisionOriginTransforms().size() == lm->getShapes().size());

        for (const auto shape_xform : boost::combine(lm->getCollisionOriginTransforms(),lm->getShapes()))
        {
            visualization_msgs::Marker mk;

            if (!shapes::constructMarkerFromShape(shape_xform.get<1>().get(),mk)) {
                ROS_WARN("Failed to construct marker.");
            }

            tf::poseEigenToMsg(xfm * shape_xform.get<0>(), mk.pose);

            mk.header.frame_id = "world";

            mk.id = ma.markers.size();

            mk.color.r = 255;
            mk.color.g = 255;
            mk.color.b = 255;
            mk.color.a = 255;

            ma.markers.push_back(mk);
        }
    }

    return ma;

}

planning_scene::PlanningScenePtr
setupPlanningScene(const moveit_msgs::PlanningScene &planning_scene_message,
                   moveit::core::RobotModelPtr &drone) {
    auto scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setPlanningSceneDiffMsg(planning_scene_message);
    // Diff message apparently can't handle partial ACM updates?
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    return scene;
}

std::vector<Apple> apples_from_connected_components(shape_msgs::Mesh apples_mesh) {
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