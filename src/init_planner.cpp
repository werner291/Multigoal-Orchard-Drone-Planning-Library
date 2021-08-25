//
// Created by werner on 8/25/21.
//

#include "BulletContinuousMotionValidator.h"
#include "make_robot.h"
#include "ClearanceDecreaseMinimzationObjective.h"
#include "build_planning_scene.h"
#include "build_request.h"
#include "EndEffectorConstraintSampler.h"
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <random_numbers/random_numbers.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/util.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include "init_planner.h"

std::shared_ptr<robowflex::OMPL::OMPLInterfacePlanner> init_planner(std::shared_ptr<robowflex::Robot> drone, std::shared_ptr<robowflex::Scene> scene) {

    auto simple_planner = std::make_shared<robowflex::OMPL::OMPLInterfacePlanner>(drone, "simple");

    robowflex::OMPL::Settings settings;
    settings.simplify_solutions = false;

    if (!simple_planner->initialize("test_robots/config/ompl_planning.yaml", settings)) {
        ROS_ERROR("Planner initialization failed.");
    }

    simple_planner->getInterface()
            .getConstraintSamplerManager()
            .registerSamplerAllocator(std::make_shared<DroneStateConstraintSamplerAllocator>());

    // Not available in standard Robowflex.
    // See: https://github.com/KavrakiLab/robowflex/pull/255
    simple_planner->setPreplanCallback([&](){
        const ompl::geometric::SimpleSetupPtr &ss = simple_planner->getLastSimpleSetup();
        ss->setOptimizationObjective(
                std::make_shared<ClearanceDecreaseMinimzationObjective>(ss->getSpaceInformation())
        );

        ss->getSpaceInformation()->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(ss->getSpaceInformation().get(), drone, scene));
    });

    return simple_planner;
}