/**
 * @file approach_planning.cpp
 *
 * The purpose of this experiment is to quantitatively compare the performance
 * of various approach path planning algorithms.
 */

#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "../utilities/experiment_utils.h"
#include "../utilities/cgal_utils.h"

#include "../pure/WorkspaceShell.h"
#include "../shell_space/CGALMeshShell.h"
#include "../planners/shell_path_planner/Construction.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../pure/approach_planning.h"
#include "../pure/ompl_impl/OmplShellSpace.h"
#include "../pure/ompl_impl/configuration_space.h"
#include "../planners/MultigoalPrmStar.h"
#include "../any_to_any_dijkstra.h"

int main(int argc, char **argv) {

	const int N_TREES = 10;
	const int MAX_FRUIT = 300;

	auto models = loadAllTreeModels(N_TREES, MAX_FRUIT);
	auto robot = loadRobotModel();

	using namespace mgodpl::approach_planning;

	using ShellPoint = mgodpl::cgal_utils::CGALMeshPointAndNormal;
	using ShellSpace = OmplShellSpace<ShellPoint>;
	using GoalRegion = ompl::base::GoalPtr;
	using Path = ompl::geometric::PathGeometric;
	using ApproachFn = ApproachPlannerFn<ShellSpace, GoalRegion, Path>;
	using BatchApproachFn = BatchApproachPlannerFn<ShellSpace, GoalRegion, Path>;

	Json::Value results;

	// Iterate over the tree models one by one.

	for (const auto &tree_models: models) {

		Json::Value tree_results;

		tree_results["n_total"] = (int) tree_models.fruit_meshes.size();

		auto scene = createSceneFromTreeModels(tree_models);

		auto ss = omplStateSpaceForDrone(robot, TRANSLATION_BOUND);
		auto si = loadSpaceInformation(ss, scene);

		const auto shell = cgalChullShell(scene, si);

		MakeshiftPrmApproachPlanningMethods<ShellPoint> methods(si);

		std::vector<ompl::base::GoalPtr> goal_regions =
				scene.apples | ranges::views::transform([&](const auto &apple) -> ompl::base::GoalPtr {
					return std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center);
				}) | ranges::to<std::vector>();

		ApproachFn makeshift = [&](const ompl::base::GoalPtr &goal_region, const ShellSpace &shell) -> std::optional<Path> {
			auto path = methods.approach_path(goal_region, shell);
			if (path.has_value()) {
				return path->robot_path;
			} else {
				return std::nullopt;
			}
		};

		BatchApproachFn batch_makeshift = adapt_individual_planner_to_batch(makeshift);

		BatchApproachFn batch_roadmap = [&](const std::vector<ompl::base::GoalPtr> &goal_regions, const ShellSpace &shell) -> std::vector<std::optional<Path>> {

			// Generate shell states based on closest pairs.

			std::vector<ShellPoint> shell_points = goal_regions | ranges::views::transform([&](const auto &goal_region) -> ShellPoint {
				return find_closest_shellpoint(shell, *goal_region);
			}) | ranges::to<std::vector>();

			// Build a roadmap.
			PRMCustom prm(si);

			auto objective = std::make_shared<DronePathLengthObjective>(si);
			auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
			pdef->setOptimizationObjective(objective);
			prm.setProblemDefinition(pdef);

			prm.constructRoadmap(ompl::base::timedPlannerTerminationCondition(1.0));

			ompl::base::ScopedState<> shell_state(si);

			std::vector<PRMCustom::Vertex> shell_vertices;

			for (const auto &shell_point: shell_points) {
				shell.stateFromPoint(shell_point, shell_state.get());
				shell_vertices.push_back(prm.insert_state(shell_state.get()));
			}

			auto time_start = std::chrono::high_resolution_clock::now();

			// Connect to the goals.
			std::vector<std::vector<PRMCustom::Vertex>> goal_vertices;
			goal_vertices.reserve(goal_regions.size());

			for (const auto &goal_region: goal_regions) {
				// FIXME: Yikes ugly cast.
				goal_vertices.push_back(prm.tryConnectGoal(*goal_region->as<ompl::base::GoalSampleableRegion>(), 5));
			}

			auto time_end = std::chrono::high_resolution_clock::now();
			std::cout << "Inserting special states took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() << "ms" << std::endl;

			// Use a variant of Dijkstra to find the shortest path from any of the shell vertices to any of the goal vertices.
			auto paths_in_graph = mgodpl::any_to_any_dijkstra::any_to_any(
					prm.getRoadmap(),
					[&](const PRMCustom::Vertex& a, const PRMCustom::Vertex& b) {
						return prm.getSpaceInformation()->distance(prm.getState(a), prm.getState(b));
					},
					shell_vertices,
					goal_vertices
					);


			std::vector<std::optional<Path>> paths;
			paths.reserve(goal_regions.size());

			for (const auto &path_in_graph: paths_in_graph) {
				if (path_in_graph.size() >= 2) {
					ompl::geometric::PathGeometric path(si);
					for (const auto &vertex: path_in_graph) {
						path.append(prm.getState(vertex));
					}
					paths.push_back(path);
				} else {
					paths.push_back(std::nullopt);
				}
			}

			return paths;

		};

		const std::vector<std::pair<std::string, BatchApproachFn>> planners = {
//				{"makeshift", batch_makeshift},
				{"roadmap", batch_roadmap}
		};

		// Iterate over the planners one by one.

		for (const auto &[planner_name, planner_fn]: planners) {

			Json::Value planner_results;

			auto start_time = std::chrono::high_resolution_clock::now();

			// Run the planner on the scene.
			auto paths = planner_fn(goal_regions, *shell);

			auto duration = std::chrono::high_resolution_clock::now() - start_time;

			// Iterate over the paths one by one.
			for (const auto &path: paths) {

				// If the path is valid, then we can do something with it.
				if (path.has_value()) {

					Json::Value path_results;
					path_results["length"] = path->length();
					planner_results["paths"].append(path_results);

				} else {

					planner_results["paths"].append(Json::Value::null);

				}

			}

			planner_results["total_planning_time"] = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

			tree_results["by_planner"][planner_name] = planner_results;

		}

		results[tree_models.tree_name] = tree_results;

	}

//	std::ofstream file("analysis/data/approach_planning.json");
//	file << results;
//	file.close();

}