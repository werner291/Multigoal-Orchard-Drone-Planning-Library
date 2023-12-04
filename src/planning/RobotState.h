// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12/4/23.
//

#include <vector>
#include "../math/Transform.h"

namespace mgodpl {

	struct RobotState
	{
		math::Transformd base_tf;
		std::vector<double> joint_values;
	};

}