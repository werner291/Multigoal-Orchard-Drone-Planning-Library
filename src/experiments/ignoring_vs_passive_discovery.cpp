#include <range/v3/all.hpp>
#include <boost/asio.hpp>

#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../planners/ShellPathPlanner.h"
#include "../vtk/Viewer.h"
#include "../CurrentPathState.h"

#include "../vtk/SimpleVtkViewer.h"
#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/goal_events.h"

#include <vtkProperty.h>

struct PlanningProblem {
	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
};

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
}

struct RecomputationEvent {
	ChangeIgnoringReplannerAdapter::GoalChanges goal_changes;
	double at_t;
};

enum DiscoveryStatus {
	VISITED, KNOWN_TO_ROBOT, EXISTS_BUT_UNKNOWN_TO_ROBOT
};

/**
 * Scan through the events and find the first recomputation event,
 * recording all the goals that were visited in the meantime as well.
 *
 * @param events            Events to scan through.
 * @param goals             Goals to look up by ID.
 * @param discovery_status  Discovery status of each goal (will be modified!)
 * @return                  Recomputation event, or nullopt if no recomputation event is found.
 */
[[nodiscard]]
std::optional<RecomputationEvent> find_recomputation_event(const std::vector<utilities::GoalEvent> &events,
														   const std::vector<ompl::base::GoalPtr> &goals,
														   std::vector<DiscoveryStatus> &discovery_status) {

	RecomputationEvent re;

	for (const auto &event: events) {

		// Check if the current event is a GoalVisit
		if (std::holds_alternative<utilities::GoalVisit>(event)) {

			const auto &visit = std::get<utilities::GoalVisit>(event);

			// Mark the goal as visited
			discovery_status[visit.goal_id] = DiscoveryStatus::VISITED;

			// Add the visited goal to the recomputation event
			re.goal_changes.visited_goals.push_back(goals[visit.goal_id]);

			// Check if the current event is a GoalSighting
		} else if (std::holds_alternative<utilities::GoalSighting>(event)) {

			const auto &discover = std::get<utilities::GoalSighting>(event);

			// Check if the goal has not been seen before by the robot
			if (discovery_status[discover.goal_id] == DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT) {

				// Mark the goal as known to the robot
				discovery_status[discover.goal_id] = DiscoveryStatus::KNOWN_TO_ROBOT;

				// Add the newly discovered goal to the recomputation event
				re.goal_changes.new_goals.push_back(goals[discover.goal_id]);

				// Return the recomputation event
				return re;

			}
		}
	}

	// No recomputation event was found
	return std::nullopt;
}


int main(int argc, char **argv) {

	TreeMeshes meshes = loadTreeMeshes("appletree");

	meshes.fruit_meshes.resize(10);

	const auto scene_msg = treeMeshesToMoveitSceneMsg(meshes);

	const auto scene = AppleTreePlanningScene{scene_msg, meshes.fruit_meshes | ranges::views::transform(appleFromMesh) |
														 ranges::to_vector};

	const auto robot = loadRobotModel();
	const int N_INITIAL_STATES = 100;

	using namespace ranges;

	boost::asio::thread_pool pool(4);

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = std::make_shared<MoveItShellSpace<Eigen::Vector3d >>(robot, workspaceShell);

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto threadLocalStateSpace = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(threadLocalStateSpace, scene);

	const auto start_state = randomStateOutsideTree(robot, 0);
	auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size(), 1.0);

	auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

	auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

		auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

		return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	};

	auto static_planner = std::make_shared<ShellPathPlanner<Eigen::Vector3d>>(shellBuilder,
																			  std::move(approach_methods),
																			  true);

	auto change_ignoring = std::make_shared<ChangeIgnoringReplannerAdapter>(static_planner);

	ompl::base::ScopedState start(si);
	threadLocalStateSpace->copyToOMPLState(start.get(), start_state);

	std::vector<ompl::base::GoalPtr> goals =
			views::ints(0, (int) scene.apples.size()) | views::filter([&](int goal_id) {
				return apple_discoverability[goal_id] == AppleDiscoverabilityType::GIVEN;
			}) | views::transform([&](int goal_id) {
				auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, scene.apples[goal_id].center);
				return std::static_pointer_cast<ompl::base::Goal>(goal);
			}) | to_vector;

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	std::vector<DiscoveryStatus> discovery_status(scene.apples.size(), DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT);

	for (auto &[discoverable, status]: views::zip(apple_discoverability, discovery_status)) {
		if (discoverable == AppleDiscoverabilityType::GIVEN) {
			status = DiscoveryStatus::KNOWN_TO_ROBOT;
		}
	}

	auto result = change_ignoring->plan(si, start.get(), goals, scene, ptc);

	RobotPath path = omplPathToRobotPath(result.combined());

	robot_trajectory::RobotTrajectory traj = robotPathToConstantSpeedRobotTrajectory(path, 1.0);

	auto events = utilities::goal_events(traj, scene.apples, 0.1, 0.5);

	auto re = find_recomputation_event(events, goals, discovery_status);

	std::vector<vtkActor *> apple_actors;

	{

		// Create a VtkRobotmodel object to visualize the robot itself.
		VtkRobotmodel robotModel(path.waypoints.front().getRobotModel(), path.waypoints.front());

		SimpleVtkViewer viewer;

		// Add the robot model to the viewer.
		viewer.addActorCollection(robotModel.getLinkActors());

		// Add the tree meshes to the viewer.
		{
			vtkNew<vtkActorCollection> actors;

			auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
			setColorsByEncoding(tree_actor, TRUNK_RGB, false);

			auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
			setColorsByEncoding(leaves_actor, LEAVES_RGB, false);

			actors->AddItem(tree_actor);
			actors->AddItem(leaves_actor);

			for (const auto &[fruit_index, fruit_mesh]: meshes.fruit_meshes | views::enumerate) {
				auto fruit_actor = createActorFromMesh(fruit_mesh);

				std::array<double, 3> rgb{0.0, 0.0, 0.0};

				if (apple_discoverability[fruit_index]) {
					rgb = {1.0, 0.0, 0.0};
				} else {
					rgb = {0.0, 0.0, 1.0};
				}

				fruit_actor->GetProperty()->SetDiffuseColor(rgb.data());

				actors->AddItem(fruit_actor);

				apple_actors.push_back(fruit_actor);
			}

			viewer.addActorCollection(actors);
		}

		double time = 0.0;

		// The "main loop" of the program, called every frame.
		auto callback = [&]() {

			time += 0.01;

			if (re && time > re->at_t) {
				moveit::core::RobotState new_start_state(robot);
				setStateToTrajectoryPoint(new_start_state, re->at_t, traj);

				ompl::base::ScopedState new_start(si);
				threadLocalStateSpace->copyToOMPLState(new_start.get(), new_start_state);

				auto result_new = change_ignoring->replan(si, new_start.get(), re->goal_changes, scene, ptc);

				RobotPath path_new = omplPathToRobotPath(result_new.combined());

				traj = robotPathToConstantSpeedRobotTrajectory(path_new, 1.0);
			}

			// Update the robot's visualization to match the current state.
			moveit::core::RobotState state(path.waypoints.front().getRobotModel());

			setStateToTrajectoryPoint(state, time, traj);

			robotModel.applyState(state);

			for (const auto &[apple_actor, apple]: views::zip(apple_actors, scene.apples)) {
				if ((state.getGlobalLinkTransform("end_effector").translation() - apple.center).norm() < 0.05) {
					apple_actor->GetProperty()->SetDiffuseColor(0.0, 1.0, 0.0);
				}
			}

		};

		viewer.addTimerCallback(callback);

		viewer.start();

	}

	return EXIT_SUCCESS;

}

