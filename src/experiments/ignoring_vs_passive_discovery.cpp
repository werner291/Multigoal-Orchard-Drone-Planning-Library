#include <range/v3/all.hpp>
#include <boost/asio.hpp>

#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../abstract/static_shell_approach.h"
#include "../abstract/approach_paths.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../utilities/traveling_salesman.h"

struct PlanningProblem {
	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
};

int main(int argc, char **argv) {

	const auto scene = createMeshBasedAppleTreePlanningSceneMessage("apple_tree", true);
	const auto robot = loadRobotModel();
	const int N_INITIAL_STATES = 100;

	using namespace ranges;

	boost::asio::thread_pool pool(4);

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));

	auto shell = std::make_shared<MoveItShellSpace<Eigen::Vector3d>>(robot, workspaceShell);

	for (int i = 0; i < N_INITIAL_STATES; i++) {
		boost::asio::post(pool, [&]() {

			const auto start_state = randomStateOutsideTree(robot, i);
			auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size());

			using GoalId = size_t;

			std::vector<GoalId> initial_goals = views::ints(0, (int) scene.apples.size()) | views::filter[&](GoalId
			goal_id) {
			return apple_discoverability[goal_id];
		} | views::transform([&](int goal_id) {
			return (GoalId) goal_id;
		}) | to_vector;

			// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
			// So, we just re-create the state space every time just to be safe.
			auto threadLocalStateSpace = omplStateSpaceForDrone(robot);

			// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
			// we'll need to copy this every time.
			auto si = loadSpaceInformation(threadLocalStateSpace, scene);

			OmplShellSpace<Eigen::Vector3d> omplShell(si, shell);

			auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

			auto plan_goal_approach = [&](const GoalId &goal) -> std::optional<abstract::ApproachPath<RobotPath, Eigen::Vector3d>> {
				auto ompl_goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.0, scene.apples[goal].center);
				auto path = approach_methods->approach_path(ompl_goal, omplShell);

				if (!path) {
					std::cout << "No path found to goal " << goal << std::endl;
					return std::nullopt;
				}

				return {
						abstract::ApproachPath<RobotPath, Eigen::Vector3d>{
								omplPathToRobotPath(path->robot_path),
								path->shell_point
						}
				};
			};

			auto plan_state_approach = [&](const moveit::core::RobotState &goal) -> std::optional<abstract::ApproachPath<RobotPath, Eigen::Vector3d>> {

				// Convert to OMPL state
				ompl::base::ScopedState state(si);

				threadLocalStateSpace->copyToOMPLState(state.get(), goal);

				auto path = approach_methods->initial_approach_path(state.get(), omplShell);

				if (!path) {
					std::cout << "No path found to goal " << goal << std::endl;
					return std::nullopt;
				}

				return {
						abstract::ApproachPath<RobotPath, Eigen::Vector3d>{
								omplPathToRobotPath(path->robot_path),
								path->shell_point
						}
				};
			};

			auto plan_shell_path = [&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> RobotPath {
				auto path = shell->shellPath(from, to);
			};

			auto tsp_planner = [&](const Eigen::Vector3d &from, const std::vector<Eigen::Vector3d>& shell_states) {

				// We'll want to invoke our favorite TSP solver on the shell states.
				return tsp_open_end(
						[&](size_t i) {
							return shell->predict_path_length(from, shell_states[i]);
						},
						[&](size_t i, size_t j) {
							return shell->predict_path_length(shell_states[i], shell_states[j]);
						},
						shell_states.size()
				);

			};

			auto path_optimizer = [&](const RobotPath &path) -> RobotPath {
				return path;
			};

			plan_path_static_goalset<moveit::core::RobotState, GoalId, RobotPath, Eigen::Vector3d>(
					start_state,
					initial_goals,
					plan_goal_approach,
					plan_state_approach,
					plan_shell_path,
					tsp_planner,
					path_optimizer
					);
		});
	}

	pool.join();

}