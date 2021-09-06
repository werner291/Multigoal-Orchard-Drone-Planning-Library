#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io.h>
#include <robowflex_library/trajectory.h>
#include "msgs_utilities.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "init_planner.h"
#include "InverseClearanceIntegralObjective.h"
#include "ClearanceDecreaseMinimizationObjective.h"
#include "EndEffectorConstraintSampler.h"
#include <fcl/fcl.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>

using namespace robowflex;

class CustomModelBasedStateSpace : public ompl_interface::ModelBasedStateSpace {

    const std::string param_type_ = "custom";
public:
    CustomModelBasedStateSpace(const ompl_interface::ModelBasedStateSpaceSpecification &spec)
            : ModelBasedStateSpace(spec) {}

    unsigned int validSegmentCount(const ompl::base::State *state1, const ompl::base::State *state2) const override {
        return this->distance(state1, state2) / 0.2;
    }

    const std::string &getParameterizationType() const override {
        return param_type_;
    }

};

class StateValidityChecker : public ompl::base::StateValidityChecker {

    robowflex::SceneConstPtr scene_;

public:
    StateValidityChecker(ompl::base::SpaceInformation *si, robowflex::SceneConstPtr scene)
            : ompl::base::StateValidityChecker(si), scene_(scene) {
    }

    bool isValid(const ompl::base::State *state) const override {

        auto space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();

        moveit::core::RobotState robot_state(space->getRobotModel());
        space->copyToRobotState(robot_state, state);

        // We rely on the sampler producing states that are valid in all other aspects, so here we just check collision.
        return !scene_->checkCollision(robot_state).collision;
    }

    double clearance(const ompl::base::State *state) const override {
        auto space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();

        moveit::core::RobotState robot_state(space->getRobotModel());
        space->copyToRobotState(robot_state, state);

        // We rely on the sampler producing states that are valid in all other aspects, so here we just check collision.
        return scene_->distanceToCollision(robot_state);
    }

};

class DroneStateSampler : public ompl::base::StateSampler {

public:
    DroneStateSampler(const ompl::base::StateSpace *space)
            : StateSampler(space) {}

    void sampleUniform(ompl::base::State *state) override {
        moveit::core::RobotState st(space_->as<CustomModelBasedStateSpace>()->getRobotModel());
        DroneStateConstraintSampler::randomizeUprightWithBase(st);
        space_->as<CustomModelBasedStateSpace>()->copyToOMPLState(state, st);
    }

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {
        ROS_ERROR("Not implemented.");
    }

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override {
        ROS_ERROR("Not implemented.");
    }
};

class InverseClearanceIntegralObjectiveOMPL : public ompl::base::StateCostIntegralObjective {
public:
    InverseClearanceIntegralObjectiveOMPL(const ompl::base::SpaceInformationPtr &si, bool enableMotionCostInterpolation)
            : StateCostIntegralObjective(si, enableMotionCostInterpolation) {}

    ompl::base::Cost stateCost(const ompl::base::State *s) const override {
        return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
    }

};

class DroneEndEffectorNearTarget : public ompl::base::GoalSampleableRegion {

    double radius;
    Eigen::Vector3d target;

public:
    DroneEndEffectorNearTarget(const ompl::base::SpaceInformationPtr &si, double radius, const Eigen::Vector3d &target)
            : GoalSampleableRegion(si), radius(radius), target(target) {}

    void sampleGoal(ompl::base::State *state) const override {
        auto *state_space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();

        moveit::core::RobotState st(state_space->getRobotModel());
        DroneStateConstraintSampler::randomizeUprightWithBase(st);
        DroneStateConstraintSampler::moveEndEffectorToGoal(st, radius, target);
        state_space->as<CustomModelBasedStateSpace>()->copyToOMPLState(state, st);
    }

    [[nodiscard]] unsigned int maxSampleCount() const override {
        return INT_MAX;
    }

    double distanceGoal(const ompl::base::State *state) const override {
        auto *state_space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();
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

    std::vector<planning_interface::MotionPlanResponse> responses;

    Trajectory full_trajectory(drone, "whole_body");
    full_trajectory.addSuffixWaypoint(genStartState(drone));

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModel(), "whole_body");

    auto state_space = std::make_shared<CustomModelBasedStateSpace>(spec);
    state_space->setStateSamplerAllocator([](const ompl::base::StateSpace *space) {
        return std::make_shared<DroneStateSampler>(space);
    });

    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);
    si->setStateValidityChecker(std::make_shared<StateValidityChecker>(si.get(), scene));

    si->setup();

    full_trajectory.addSuffixWaypoint(genStartState(drone));

    ompl::geometric::PRM prm(si);

    auto avoid_branches = std::make_shared<InverseClearanceIntegralObjectiveOMPL>(si, false);

    std::cout << "Building PRM" << std::endl;

    for (const Apple &apple: tree_scene.apples) {

        ompl::base::ScopedState start(si);
        state_space->copyToOMPLState(start.get(), full_trajectory.getTrajectory()->getLastWayPoint());

        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef->addStartState(start);
//        pdef->setOptimizationObjective(avoid_branches);

        pdef->setGoal(std::make_shared<DroneEndEffectorNearTarget>(si, 0.2, apple.center));

        prm.setProblemDefinition(pdef);

        ompl::base::PlannerStatus status = prm.solve(ompl::base::timedPlannerTerminationCondition(5.0));

        if (status) {
            auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
            ompl::base::State *last_state = nullptr;
            for (auto state: path->getStates()) {
                state_space->copyToRobotState(*drone->getScratchState(), state);
                full_trajectory.addSuffixWaypoint(*drone->getScratchState());
                last_state = state;
            }
            std::cout << "Point-to-point solution found." << std::endl;
        } else {
            std::cout << "Apple unreachable" << std::endl;
        }

        prm.clearQuery();
    }

    full_trajectory.interpolate(20 * full_trajectory.getNumWaypoints());

    rviz.updateTrajectory(full_trajectory);

    return 0;
}


