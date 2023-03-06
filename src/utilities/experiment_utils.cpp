
#include <utility>
#include <fstream>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <range/v3/all.hpp>
#include "../SingleGoalPlannerMethods.h"
//#include "../src/NewKnnPlanner.h"
#include "../probe_retreat_move.h"
#include "../GreatCircleMetric.h"
#include "experiment_utils.h"
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

#include "Seb.h"

#include "../planning_scene_diff_message.h"
#include "msgs_utilities.h"
#include "experiment_utils.h"
#include "json_utils.h"
#include "general_utilities.h"
#include "../DroneStateConstraintSampler.h"
#include "mesh_utils.h"


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

moveit::core::RobotModelPtr loadRobotModel(double base_joint_weight) {

	auto urdf = std::make_shared<urdf::Model>();
	urdf->initFile("test_robots/urdf/bot.urdf");

	auto srdf = std::make_shared<srdf::Model>();
	srdf->initFile(*urdf, "test_robots/config/aerial_manipulator_drone.srdf");

	auto robot = std::make_shared<moveit::core::RobotModel>(urdf, srdf);

	// By default, the joint distance factor gets set to the dimension count of the joint, see:
	//     https://github.com/ros-planning/moveit/blob/fd36674cc327962aaf27925ddf1ba9c6a8667d35/moveit_core/robot_model/src/robot_model.cpp#L969
	// I have no idea why this is, but I don't like it.
	for (auto &item: robot->getActiveJointModels())
		item->setDistanceFactor(1.0);

	// If it crashes here, it's because the file isn't found.
	robot->getJointModel("world_joint")->setDistanceFactor(base_joint_weight);

	return robot;
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


std::optional<ompl::geometric::PathGeometric>
planFromStateToState(ompl::base::Planner &planner, const ompl::base::OptimizationObjectivePtr &objective,
                     const ompl::base::State *a, const ompl::base::State *b, double duration) {

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner.getSpaceInformation());
    pdef->setOptimizationObjective(objective);
    pdef->setStartAndGoalStates(a,b);
    planner.setProblemDefinition(pdef);
    if (planner.solve(ompl::base::timedPlannerTerminationCondition(duration)) == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        return {*pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>()};
    } else {
        return {};
    }
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

moveit::core::RobotState randomStateOutsideTree(const moveit::core::RobotModelConstPtr &drone, const int seed) {
    moveit::core::RobotState start_state(drone);

    randomizeUprightWithBase(start_state, 0.0);

    ompl::RNG rng(seed);

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


std::vector<moveit::core::RobotState> randomStatesOutsideTree(const AppleTreePlanningScene &scene,
															  const moveit::core::RobotModelConstPtr &model,
															  const int n) {

	using namespace ranges;

	return views::iota(0, n) | views::transform([&](int i) {
		return randomStateOutsideTree(model, i);
	}) | to_vector;

}

ompl::base::SpaceInformationPtr
loadSpaceInformation(const std::shared_ptr<DroneStateSpace> &stateSpace, const AppleTreePlanningScene &scene_info) {
	auto scene = setupPlanningScene(*scene_info.scene_msg, stateSpace->getRobotModel());
	return initSpaceInformation(scene, scene->getRobotModel(), stateSpace);
}

std::shared_ptr<DroneStateSpace> omplStateSpaceForDrone(const moveit::core::RobotModelConstPtr &model) {
	ompl_interface::ModelBasedStateSpaceSpecification spec(model, "whole_body");
	return std::make_shared<DroneStateSpace>(spec, TRANSLATION_BOUND);
}

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
}

