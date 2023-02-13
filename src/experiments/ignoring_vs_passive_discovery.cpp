#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../abstract/static_shell_approach.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"

enum AppleDiscoverabilityType {
	GIVEN, DISCOVERABLE
};


struct PlanningProblem {


	moveit::core::RobotState start_state;


};

/**
 * @brief Generates a set of booleans indicating the discoverability of each apple
 *
 * @param num_apples The number of apples in the scene
 * @return std::vector<bool> A vector of booleans indicating the discoverability of each apple
 */
std::vector<bool> generateAppleDiscoverability(int num_apples) {
	auto rng = std::default_random_engine(num_apples);

	double p = std::uniform_real_distribution<double>(0.1, 1.0)(rng);

	std::vector<bool> apple_discoverability(num_apples);

	std::generate(apple_discoverability.begin(), apple_discoverability.end(), [&]() {
		return std::bernoulli_distribution(p)(rng);
	});

	return apple_discoverability;
}

int main(int argc, char **argv) {

	const auto scene = createMeshBasedAppleTreePlanningSceneMessage("apple_tree", true);
	const auto robot = loadRobotModel();
	const int N_INITIAL_STATES = 100;

	using namespace ranges;

	boost::asio::thread_pool pool(4);

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));

	MoveItShellSpace<Eigen::Vector3d> shell(robot, workspaceShell);


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

			auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

			// std::function<std::optional<Path>(const State&, const Goal&, const ShellSpace&)>


			plan_path_static_goalset<moveit::core::RobotState, GoalId, RobotPath, Eigen::Vector3d>(start_state,
																								   initial_goals,
																								   [&](const auto &goal,
																									   const auto &shell_space) {
																									   return RobotPath{};
																								   },
																								   [&](const auto &state,
																									   const auto &shell_space) {
																									   return RobotPath{};
																								   },
																								   [&](const auto &from,
																									   const auto &to) {
																									   return RobotPath{};
																								   },
																								   [&](const auto &start,
																									   const auto &shell_points) -> std::vector<size_t> {
																									   return {};
																								   },
																								   [&](const auto &path) {
																									   return path;
																								   });
		});
	}

	pool.join();

}