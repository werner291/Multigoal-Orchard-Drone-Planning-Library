#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io.h>
#include <robowflex_library/trajectory.h>
#include "build_request.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "init_planner.h"
#include "InverseClearanceIntegralObjective.h"
#include "ClearanceDecreaseMinimizationObjective.h"
#include "EndEffectorConstraintSampler.h"
#include <fcl/fcl.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>

using namespace robowflex;

class DroneStateSampler : public ompl::base::StateSampler {

public:
    DroneStateSampler(const ompl::base::StateSpace *space)
            : StateSampler(space) {}

    void sampleUniform(ompl::base::State *state) override {
        moveit::core::RobotState st(space_->as<ompl_interface::ModelBasedStateSpace>()->getRobotModel());
        DroneStateConstraintSampler::randomizeUprightWithBase(st);
        space_->as<ompl_interface::ModelBasedStateSpace>()->copyToOMPLState(state, st);
    }

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {
        ROS_ERROR("Not implemented.");
    }

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override {
        ROS_ERROR("Not implemented.");
    }
};

class DroneEndEffectorNearTarget : public ompl::base::GoalSampleableRegion {

    double radius;
    Eigen::Vector3d target;

public:
    DroneEndEffectorNearTarget(const ompl::base::SpaceInformationPtr &si, double radius, const Eigen::Vector3d &target)
            : GoalSampleableRegion(si), radius(radius), target(target) {}

    void sampleGoal(ompl::base::State *state) const override {
        auto *state_space = si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>();

        moveit::core::RobotState st(state_space->getRobotModel());
        DroneStateConstraintSampler::randomizeUprightWithBase(st);
        DroneStateConstraintSampler::moveEndEffectorToGoal(st, radius, target);
        state_space->as<ompl_interface::ModelBasedStateSpace>()->copyToOMPLState(state, st);
    }

    [[nodiscard]] unsigned int maxSampleCount() const override {
        return INT_MAX;
    }

    double distanceGoal(const ompl::base::State *state) const override {
        auto *state_space = si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>();
        moveit::core::RobotState st(state_space->getRobotModel());

        Eigen::Vector3d ee_pos = st.getGlobalLinkTransform("end_effector").translation();

        Eigen::Vector3d delta = target - ee_pos;

        return delta.norm();
    }

};

/**
 * The "visualized" version of this program, which serves as a scratch state in which to experiment with new,
 * and potentially useless changes.
 *
 * See the benchmark main() method for the more reproducible results.
 */
int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    std::shared_ptr<Robot> drone = make_robot();

    IO::RVIZHelper rviz(drone);
    IO::RobotBroadcaster bc(drone);
    bc.start();

    auto scene = std::make_shared<Scene>(drone);
    auto tree_scene = establishPlanningScene(10);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    auto optimizationObjectiveAllocator = [](const ompl::geometric::SimpleSetupPtr &ss) {
        return std::make_shared<ClearanceDecreaseMinimizationObjective>(ss->getSpaceInformation());
    };

    std::vector<planning_interface::MotionPlanResponse> responses;

    Trajectory full_trajectory(drone, "whole_body");
    full_trajectory.addSuffixWaypoint(genStartState(drone));

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModel(), "whole_body");

    auto state_space = std::make_shared<ompl_interface::JointModelStateSpace>(spec);
    state_space->setStateSamplerAllocator([](const ompl::base::StateSpace* space) {
        return std::make_shared<DroneStateSampler>(space);
    });

    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);
    si->setup();

    ompl::base::ScopedState start(si);
    state_space->copyToOMPLState(start.get(), genStartState(drone));

    ompl::geometric::PRM prm(si);

    std::cout << "Building PRM" << std::endl;

    prm.constructRoadmap(ompl::base::timedPlannerTerminationCondition(1.0));

//    auto simple_planner = init_planner(drone, scene, optimizationObjectiveAllocator);

    for (const Apple& apple : tree_scene.apples) {

        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef->addStartState(start);

        pdef->setGoal(std::make_shared<DroneEndEffectorNearTarget>(si, 0.2, apple.center));

        prm.setProblemDefinition(pdef);

        prm.solve(ompl::base::timedPlannerTerminationCondition(0.1));

//        moveit_msgs::MotionPlanRequest request =
//                makeAppleReachRequest(drone, "PRMStar", 0.1,
//                                      apple,
//                                      full_trajectory.getTrajectory()->getLastWayPoint());

//        simple_planner->refreshContext(scene, request, true);
//        auto response = simple_planner->plan(scene, request);

//        if (response.error_code_.val == 1) {
//
//            for (size_t wpi = 0; wpi < response.trajectory_->getWayPointCount(); wpi++) {
//                full_trajectory.addSuffixWaypoint(response.trajectory_->getWayPoint(wpi));
//            }
//        } else {
//            rviz.addGoalMarker("goal_request_marker", request);
//        }
    }

//    rviz.updateTrajectory(full_trajectory);

//    std::cin.get();

    return 0;
}


