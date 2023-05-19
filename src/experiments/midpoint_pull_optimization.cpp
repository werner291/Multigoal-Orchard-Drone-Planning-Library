// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "../utilities/experiment_utils.h"
#include "../quickplan.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../utilities/midpoint_pull.h"
#include "../DroneStateConstraintSampler.h"

void projectgoalRegion(ompl::base::State *st, const ompl::base::GoalPtr &goal_base) {

	auto goal = std::dynamic_pointer_cast<DroneEndEffectorNearTarget>(goal_base);

	auto *state_space = goal->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

	moveit::core::RobotState moveit_st(state_space->getRobotModel());
	state_space->copyToRobotState(moveit_st, st);

	moveEndEffectorToGoal(moveit_st, 0.0, goal->getTarget());

	state_space->copyToOMPLState(st, moveit_st);

}

int main() {

	auto robot = loadRobotModel();
	auto models = loadTreeMeshes("appletree");

	// Shuffle the apple meshes and take 20 of them
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(models.fruit_meshes.begin(), models.fruit_meshes.end(), g);
	//	models.fruit_meshes.resize(20);

	auto start_state = randomStateOutsideTree(robot, 42);

	auto scene = createSceneFromTreeModels(models);

	auto ss = omplStateSpaceForDrone(start_state.getRobotModel());
	auto si = loadSpaceInformation(ss, scene);
	auto planner = makeShellBasedPlanner<CGALMeshShellPoint>(cgalChullShell)(si);
	ompl::base::ScopedState<> start(ss);
	ss->copyToOMPLState(start.get(), start_state);
	auto ptc = ompl::base::plannerNonTerminatingCondition();
	auto goals = appleGoalsFromScene(scene, si);

	auto plan_result = planner->plan(si, start.get(), goals, scene, ptc);

	for (auto &item: plan_result.segments) {
		item.path_.subdivide();
	}

	SimpleVtkViewer viz;
	addTreeMeshesToViewer(viz, models);

	auto path = plan_result.combined();

	double length_before = path.length();

	std::cout << "Path length before: " << length_before << std::endl;

	double t = 1.0;  // Start with t = 1.0
	double lastLength = length_before;  // Keep track of the last path length
	double improvementThreshold = 0.01 * length_before; // Set improvement threshold to 1% of the original path length
	double absoluteImprovement; // Absolute improvement in path length

	do {
		bool improved = midpointPullTryShortenPath(path,
												   extractGoalIndexPairing(plan_result, goals),
												   projectgoalRegion,
												   t);
		double currentLength = path.length();

		// Calculate the absolute improvement
		absoluteImprovement = lastLength - currentLength;

		std::cout << "Path length after: " << currentLength << std::endl;

		// If the path length didn't decrease, or the absolute improvement is less than the threshold, halve t
		if (!improved || absoluteImprovement < improvementThreshold) {
			t *= 0.5;
		}

		// Update the last path length
		lastLength = currentLength;

		// If t has become very small, break the loop
		if (t < 1e-6) {
			break;
		}
	} while (absoluteImprovement >=
			 improvementThreshold);  // Stop when the absolute improvement falls below the threshold

	auto rpath_moveit = omplPathToRobotPath(path);

	visualizeBaseEndEffectorLadderTrace(viz, rpath_moveit);

	viz.start();

	return 0;
}