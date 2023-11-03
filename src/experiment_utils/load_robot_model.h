// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#ifndef MGODPL_LOAD_ROBOT_MODEL_H
#define MGODPL_LOAD_ROBOT_MODEL_H

#include <memory>
#include "../planning/moveit_forward_declarations.h"

namespace mgodpl::experiment_assets {

	moveit::core::RobotModelPtr loadRobotModel(double base_joint_weight);

}

#endif //MGODPL_LOAD_ROBOT_MODEL_H
