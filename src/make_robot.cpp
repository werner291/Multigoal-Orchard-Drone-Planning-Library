//
// Created by werner on 8/24/21.
//

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
