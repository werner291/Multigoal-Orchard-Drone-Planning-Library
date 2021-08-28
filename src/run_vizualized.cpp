#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include "build_request.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "init_planner.h"
#include "MyCollisionDetectorAllocatorBullet.h"
#include "InverseClearanceIntegralObjective.h"

using namespace robowflex;

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
    auto tree_scene = establishPlanningScene(8);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    scene->getScene()->setActiveCollisionDetector(MyCollisionDetectorAllocatorBullet::create(),
                                                  true);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    moveit_msgs::MotionPlanRequest request = makeAppleReachRequest(drone, tree_scene.apples, "RRTX", 60.0);



    rviz.addGoalMarker("goal_request_marker", request);

    rviz.updateMarkers();

    auto optimizationObjectiveAllocator = [](const ompl::geometric::SimpleSetupPtr &ss) {
        return std::make_shared<InverseClearanceIntegralObjective>(ss->getSpaceInformation(), false);
    };

    auto simple_planner = init_planner(drone, scene, optimizationObjectiveAllocator);

    auto response = simple_planner->plan(scene, request);


    if (response.error_code_.val == 1) {

//        simple_planner->getLastSimpleSetup()->getPathSimplifier()->smoothBSpline()

        rviz.updateTrajectory(response.trajectory_);

        auto opt = optimizationObjectiveAllocator(simple_planner->getLastSimpleSetup());
        auto mbsp = simple_planner->getLastSimpleSetup()->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>();

        ompl::base::ScopedState st1(simple_planner->getLastSimpleSetup()->getSpaceInformation());
//        ompl::base::ScopedState st2(simple_planner->getLastSimpleSetup()->getSpaceInformation());

        for (size_t i = 0; i < response.trajectory_->getWayPointCount(); i++) {
            mbsp->copyToOMPLState(st1.get(), response.trajectory_->getWayPoint(i));
//            mbsp->copyToOMPLState(st2.get(), response.trajectory_->getWayPoint(i+1));

//              opt->stateCost(st1.get());
            double clearance = 1.0 / simple_planner->getLastSimpleSetup()->getStateValidityChecker()->clearance(st1.get());

            std::cout << "Clearance: " << i << " - " << clearance << std::endl;

            std::ostringstream out;
            out.precision(1);
            out << std::fixed << clearance;


            rviz.addTextMarker("clearance_label",
                               out.str(),
                               "world",
                               response.trajectory_->getWayPoint(i).getGlobalLinkTransform("base_link"),
                               0.5);
//            opt->state
//            opt->motionCost(st1.get(), st2.get());
        }

        rviz.updateMarkers();

    }

    std::cin.get();

    return 0;
}


