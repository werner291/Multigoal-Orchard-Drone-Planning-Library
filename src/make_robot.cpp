//
// Created by werner on 8/24/21.
//

#include "ClearanceDecreaseMinimizationObjective.h"
#include "build_planning_scene.h"
#include "build_request.h"
#include "EndEffectorConstraintSampler.h"
#include <random_numbers/random_numbers.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/util.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include "make_robot.h"

std::shared_ptr<robowflex::Robot> make_robot() {
    auto drone = std::make_shared<robowflex::Robot>("drone_complex");

    drone->initialize(
            "test_robots/urdf/bot.urdf",
            "test_robots/config/aerial_manipulator_drone.srdf",
            "",
            ""
    );
    return drone;
}
