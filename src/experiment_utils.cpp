
#include "../src/experiment_utils.h"
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <cstddef>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include "../src/BulletContinuousMotionValidator.h"
#include "experiment_utils.h"
#include "json_utils.h"
#include "planning_scene_diff_message.h"
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

robot_state::RobotState genStartState(const moveit::core::RobotModelConstPtr &drone) {
    robot_state::RobotState start_state(drone);
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(drone->getJointModelGroup("whole_body"),
                                       {
                                               -10.0, -10.0, 10.0,
                                               0.0, 0.0, 0.0, 1.0,
                                               0.0, 0.0, 0.0, 0.0
                                       });
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

    return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>>
constructAppleGoals(TreePlanningScene &tree_scene, const std::shared_ptr<ompl::base::SpaceInformation> &si) {
    static const double GOAL_END_EFFECTOR_RADIUS = 0.01;

    std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> goals;
    for (const auto &apple: tree_scene.apples)
        goals.push_back(
                std::make_shared<DroneEndEffectorNearTarget>(si, GOAL_END_EFFECTOR_RADIUS, apple.center));
    return goals;
}

