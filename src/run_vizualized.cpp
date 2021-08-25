#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <random_numbers/random_numbers.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include "EndEffectorConstraintSampler.h"
#include "build_request.h"
#include "build_planning_scene.h"
#include "ClearanceDecreaseMinimzationObjective.h"
#include "make_robot.h"

using namespace robowflex;

class BulletContinuousMotionValidator : public ompl::base::MotionValidator {

    std::shared_ptr<Scene> rb_scene_;
    std::shared_ptr<Robot> rb_robot_;
public:
    BulletContinuousMotionValidator(ompl::base::SpaceInformation *si,
                                    const std::shared_ptr<Robot> &rbRobot,
                                    const std::shared_ptr<Scene> &rbScene)
            : MotionValidator(si), rb_robot_(rbRobot), rb_scene_(rbScene) {}

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override {

        collision_detection::CollisionResult res;
        collision_detection::CollisionRequest req;

        auto st1 = rb_robot_->allocState();
        si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st1, s1);

        auto st2 = rb_robot_->allocState();
        si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st2, s2);

        rb_scene_->getScene()->getCollisionEnv()->checkRobotCollision(req, res, *st1, *st2, rb_scene_->getACM());

        return ! res.collision;
    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override {
        ROS_ERROR("Not implemented.");

        return false;
    }

};

int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    std::shared_ptr<Robot> drone = make_robot();

    IO::RVIZHelper rviz(drone);
    IO::RobotBroadcaster bc(drone);
    bc.start();

    auto scene = std::make_shared<Scene>(drone);
    auto tree_scene = establishPlanningScene();
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                                  true);

    auto simple_planner = std::make_shared<OMPL::OMPLInterfacePlanner>(drone, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;

    if (!simple_planner->initialize("test_robots/config/ompl_planning.yaml", settings)) {
        std::cout << "Planner initialization failed." << std::endl;
        return 1;
    }

    simple_planner->getInterface()
            .getConstraintSamplerManager()
            .registerSamplerAllocator(std::make_shared<DroneStateConstraintSamplerAllocator>());

    simple_planner->setPreplanCallback([&](){

        const ompl::geometric::SimpleSetupPtr &ss = simple_planner->getLastSimpleSetup();
        ss->setOptimizationObjective(
                std::make_shared<ClearanceDecreaseMinimzationObjective>(ss->getSpaceInformation())
                );

        ss->getSpaceInformation()->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(ss->getSpaceInformation().get(), drone, scene));

        std::cout << "Objective set." << std::endl;
    });



    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    moveit_msgs::MotionPlanRequest request = makeAppleReachRequest(drone, tree_scene.apples, "BiTRRT");

    rviz.addGoalMarker("goal_request_marker", request);

    rviz.updateMarkers();

    auto response = simple_planner->plan(scene, request);

    if (response.error_code_.val == 1) {
        rviz.updateTrajectory(response.trajectory_);
    }

    std::cin.get();

    return 0;
}


