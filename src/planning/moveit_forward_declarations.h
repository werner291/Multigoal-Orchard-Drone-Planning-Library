#pragma once

#include <memory>

namespace moveit::core {
	class RobotModel;
	using RobotModelPtr = std::shared_ptr<RobotModel>;
	using RobotModelConstPtr = std::shared_ptr<const RobotModel>;
	class RobotState;
}