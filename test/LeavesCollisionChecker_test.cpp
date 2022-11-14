
#include <gtest/gtest.h>
#include "../src/utilities/experiment_utils.h"
#include "../src/LeavesCollisionChecker.h"

TEST(LeavesCollisionChecker_test, trivial_minimal) {

	auto drone = loadRobotModel();

	LeavesCollisionChecker checker({
					{0.0, 0.0, 1.0},
					{-1.0, 0.0, 0.0},
					{1.0, 0.0, 0.0}
	});

	{
		moveit::core::RobotState state(drone);
		state.setVariablePositions({0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0});

		EXPECT_FALSE(checker.checkLeafCollisions(state).empty());
	}


	{
		moveit::core::RobotState state(drone);
		state.setVariablePositions({0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0});

		EXPECT_TRUE(checker.checkLeafCollisions(state).empty());
	}

}