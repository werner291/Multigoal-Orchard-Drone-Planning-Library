#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <random_numbers/random_numbers.h>
#include <Eigen/Geometry>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include "procedural_tree_generation.h"
#include "build_request.h"
#include "EndEffectorConstraintSampler.h"
#include "build_planning_scene.h"
#include "ClearanceDecreaseMinimzationObjective.h"
#include "make_robot.h"
#include "init_planner.h"

using namespace robowflex;

int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    auto drone = make_robot();

    auto simple_planner = init_planner();

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS | Profiler::CLEARANCE;
    Experiment experiment("pick_apple", options, 0.5, 2);

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    log::showUpToWarning();

    for (int scene_idx = 0; scene_idx < 10; scene_idx++) {
        auto scene = std::make_shared<Scene>(drone);
        auto tree_scene = establishPlanningScene(7);
        scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

        scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                                      true);

        for (const auto &config : {"PRM", "RRTConnect", "TRRT", "BiTRRT"}) {
            for (int i = 0; i < 10; i++) {
                experiment.addQuery(config, scene, simple_planner,
                                    makeAppleReachRequest(drone, tree_scene.apples, config, 10.0));
            }
        }
    }

    auto dataset = experiment.benchmark(1);

    OMPLPlanDataSetOutputter output("robowflex_drone_ompl");
    output.dump(*dataset);

    return 0;
}
