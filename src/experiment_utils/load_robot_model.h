// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#ifndef MGODPL_LOAD_ROBOT_MODEL_H
#define MGODPL_LOAD_ROBOT_MODEL_H

#include <memory>

namespace moveit::core {
	class RobotModel;
	using RobotModelPtr = std::shared_ptr<RobotModel>;
}

namespace mgodpl::experiment_assets {

	moveit::core::RobotModelPtr loadRobotModel(double base_joint_weight);

}

#endif //MGODPL_LOAD_ROBOT_MODEL_H
