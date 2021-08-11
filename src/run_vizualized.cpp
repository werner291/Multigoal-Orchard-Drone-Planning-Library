#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <random_numbers/random_numbers.h>
#include <Eigen/Geometry>
#include <moveit/robot_state/conversions.h>
#include "procedural_tree_generation.h"

using namespace robowflex;

class EndEffectorPositionConstraintSampler : public constraint_samplers::ConstraintSampler {

    std::string name_ = "";
    std::string ee_name_ = "";

    Eigen::Vector3d ee_target_;

public:
    EndEffectorPositionConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene,
                                         const std::string &groupName) :
                                         ConstraintSampler(scene, groupName) {}

    bool configure(const moveit_msgs::Constraints &constr) override {

        // TODO Some defensive coding.
        ee_target_ = Eigen::Vector3d(
                constr.position_constraints[0].constraint_region.primitive_poses[0].position.x,
                constr.position_constraints[0].constraint_region.primitive_poses[0].position.y,
                constr.position_constraints[0].constraint_region.primitive_poses[0].position.z
                );

        ee_name_ = constr.position_constraints[0].link_name;

        name_ = ee_name_ + "_position_with_mobile_base";

        return true;
    }

    bool sample(moveit::core::RobotState &state, const moveit::core::RobotState &reference_state,
                unsigned int max_attempts) override {

        state.setToRandomPositions();

        return project(state, max_attempts);
    }

    bool project(moveit::core::RobotState &state, unsigned int max_attempts) override {

        Eigen::Vector3d ee_pos = state.getGlobalLinkTransform(ee_name_).translation();

        Eigen::Vector3d delta = ee_target_ - ee_pos;

        double* positions = state.getVariablePositions();

        positions[0] += delta.x();
        positions[1] += delta.y();
        positions[2] += delta.z();

        state.update(true);

        return true;
    }

    const std::string &getName() const override {
        return name_;
    }

};

class EndEffectorPositionConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator {
    constraint_samplers::ConstraintSamplerPtr
    alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
          const moveit_msgs::Constraints &constr) override {

        auto sampler = std::make_shared<EndEffectorPositionConstraintSampler>(scene, group_name);

        sampler->configure(constr);

        return sampler;
    }

    [[nodiscard]] bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
                    const moveit_msgs::Constraints &constr) const override {
        return constr.position_constraints.size() == 1 && constr.position_constraints[0].link_name == "end_effector";
    }

};

int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    auto drone = std::make_shared<Robot>("drone");

    drone->initialize(
            "package://drone_moveit_config/urdf/bot.urdf",
            "package://drone_moveit_config/config/aerial_manipulator_drone.srdf",
            "",
            ""
    );

    IO::RVIZHelper rviz(drone);
    IO::RobotBroadcaster bc(drone);
    bc.start();

    auto scene = std::make_shared<Scene>(drone);

    auto tree_scene = establishPlanningScene();

    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    auto simple_planner = std::make_shared<OMPL::OMPLInterfacePlanner>(drone, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;
    // FIXME: see https://github.com/KavrakiLab/robowflex/issues/239
    // settings.max_goal_samples = 1;

    if (!simple_planner->initialize("package://drone_moveit_config/config/ompl_planning.yaml", settings)) {
        std::cout << "Planner initialization failed." << std::endl;
        return 1;
    }

    simple_planner->getInterface()
            .getConstraintSamplerManager()
            .registerSamplerAllocator(std::make_shared<EndEffectorPositionConstraintSamplerAllocator>());

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("pick_apple", options, 10.0, 100);

//    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

//    auto request = std::make_shared<MotionRequestBuilder>(simple_planner, "whole_body");

    moveit_msgs::MotionPlanRequest request;

    request.planner_id = "RRT";
    request.group_name = "whole_body";

    request.workspace_parameters.min_corner.x = -20.0;
    request.workspace_parameters.min_corner.y = -20.0;
    request.workspace_parameters.min_corner.z = -20.0;
    request.workspace_parameters.max_corner.x = 20.0;
    request.workspace_parameters.max_corner.y = 20.0;
    request.workspace_parameters.max_corner.z = 20.0;

    robot_state::RobotState start_state(drone->getModelConst());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(drone->getModelConst()->getJointModelGroup("whole_body"), {-10.0, -10.0, 10.0, 0.0, 0.0, 0.0, 1.0, 0.5});
    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);

    // setup a random engine
    std::default_random_engine rng(std::random_device{}());

    // setup a uniform distribution
    std::uniform_int_distribution<size_t> dis(0, tree_scene.apples.size() - 1);

    Apple apple = tree_scene.apples[dis(rng)];

    Eigen::Vector3d ee_pos(0.0,0.0,15.0);// = apple.center + apple.branch_normal;
    Eigen::Quaterniond ee_rot;
    ee_rot.setFromTwoVectors(Eigen::Vector3d(0.0,1.0,0.0),Eigen::Vector3d(apple.branch_normal.x(), apple.branch_normal.y(), 0.0));

    Eigen::Isometry3d iso;
    iso.setIdentity();
    iso.translate(ee_pos);
    iso.rotate(ee_rot);

//    RobotPose pose(iso);

    moveit_msgs::Constraints goal_constraints;
    moveit_msgs::PositionConstraint positionConstraint = TF::getPositionConstraint(
            "end_effector", "world", iso, Geometry::makeSphere(0.5));
    goal_constraints.position_constraints.push_back(positionConstraint);
    request.goal_constraints.push_back(goal_constraints);

    auto response = simple_planner->plan(scene, request);

    if (response.error_code_.val == 1) {
        rviz.updateTrajectory(response.trajectory_);
    }

    std::cin.get();

    return 0;
}
