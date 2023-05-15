#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../DynamicGoalsetPlanningProblem.h"
#include "../exploration/ColorEncoding.h"
#include "../planner_allocators.h"
#include "../planners/CachingDynamicPlanner.h"
#include "../utilities/run_experiments.h"

#include "../static_problem_generation.h"
#include "../run_static.h"

std::vector<std::pair<Json::Value, Problem>>
generateStaticOrchardPlanningProblems(const moveit::core::RobotModelPtr &robot,
									  int num_reps,
									  const std::vector<std::string> &model_names,
									  int n_per_scene) {

	const auto scenes = scenes_for_trees(model_names);

	// Generate a range of repetition indices.
	auto repIds = ranges::views::iota(0, num_reps);

	// Preallocate a vector of problems.
	std::vector<std::pair<Json::Value, Problem>> problems;

	// For each repetition...
	for (int repId : repIds) {

		// Pick 5 scenes at random.

		std::vector<AppleTreePlanningScene> reduced_scenes;

		std::sample(scenes.begin(), scenes.end(), std::back_inserter(reduced_scenes), n_per_scene, std::mt19937(repId));

		// Create a combined planning scene:

		AppleTreePlanningScene combined_scene;
		combined_scene.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>();

		double offset = 0.0;

		for (const auto &scene : reduced_scenes) {
			combined_scene.scene_msg->name += scene.scene_msg->name + "_";

			for (auto collision_object : scene.scene_msg->world.collision_objects) {
				collision_object.pose.position.x += offset;
				combined_scene.scene_msg->world.collision_objects.push_back(collision_object);
			}

			for (auto apple : scene.apples) {
				apple.center.x() += offset;
				combined_scene.apples.push_back(apple);
			}

			offset += 2.0;
		}

		// Get a random start state.
		// Careful: the trees are elongated here, so we need something custom.

		moveit::core::RobotState start_state(robot);
		start_state.setToRandomPositions(); // This is a starting point, but we need to ensure the robot is upright and outside the orchard row.

		// For simplicity, we'll put the robot at the head of the row; why would it start in the middle?

		// Get a rng
		std::mt19937 rng(repId);
		// And a 0-1 distribution
		std::uniform_real_distribution<double> dist(0.0, 1.0);

		double theta = dist(rng) * M_PI;
		double height = 0.5 + dist(rng) * 1.0;

		Eigen::Vector3d base_translation(-3.0, 0.0, 2.0);
		Eigen::Quaterniond base_rotation(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));

		Eigen::Isometry3d base_pose;
		base_pose.setIdentity();
		base_pose.translate(base_translation);
		base_pose.rotate(base_rotation);

		start_state.setJointPositions("world_joint", base_pose);
		start_state.update();

		std::cout << "Base translation: " << start_state.getGlobalLinkTransform("base_link").translation() << std::endl;
		std::cout << "Up vector: (" << start_state.getGlobalLinkTransform("base_link").rotation() * Eigen::Vector3d::UnitZ() << ")" << std::endl;

		// I'm kinda tempted to start writing the state into the planning scene as well since there's apparently a slot for it.

		Problem problem {
			.start = start_state,
			.scene = combined_scene,
		};

		Json::Value problem_json;
		problem_json["scene"] = combined_scene.scene_msg->name;
		problem_json["n_apples"] = combined_scene.apples.size();

		problems.emplace_back(problem_json, problem);

	}

	return problems;

}

int main(int argc, char **argv) {

	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	// Load the robot model.
	const auto robot = loadRobotModel();

	auto model_names = getTreeModelNames();

	// Truncate to 5 trees for testing.
	model_names.resize(10);

	const auto orchard_problems = generateStaticOrchardPlanningProblems(robot, 5, model_names, 5);

//	const auto problems = generateStaticPlanningProblems(robot, 10, model_names);

	using namespace ranges;

	// The different planners to test.
	std::vector<std::pair<Json::Value, StaticPlannerAllocatorFn>> planners = {
			{Json::Value("sphere"), makeShellBasedPlanner<Eigen::Vector3d>(omplSphereShell)},
			{Json::Value("cuttingplane"), makeShellBasedPlanner<ConvexHullPoint>(cuttingPlaneChullShell)},
			{Json::Value("cgal"), makeShellBasedPlanner<CGALMeshShellPoint>(cgalPlaneChullShell)},
			{Json::Value("cylinder"), makeShellBasedPlanner<CylinderShellPoint>(cylinderShell)}
	};

	// Run the experiments in parallel.
	runPlannersOnProblemsParallelRecoverable<StaticPlannerAllocatorFn, Problem>(planners,
																				orchard_problems,
																				runPlannerOnStaticProblem,
																				"analysis/data/shell_comparison_orchard_RAL.json",
																				8,
																				4,
																				42);

	return 0;
}
