/**
 * @file approach_planning.cpp
 *
 * The purpose of this experiment is to quantitatively compare the performance
 * of various approach path planning algorithms.
 */

#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

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

std::vector<std::vector<PRMCustom::Vertex>> any_to_any(
		const PRMCustom &graph,
		std::vector<PRMCustom::Vertex> start_points,
		std::vector<std::vector<PRMCustom::Vertex>> goal_groups) {

	/**
	 * A vertex in the roadmap with a parent vertex and a cost.
	 */
	struct Parent {
		// The best parent vertex. (nullopt if this is the start vertex or if the node has not been visited.)
		std::optional<PRMCustom::Vertex> best_parent;
		// The cost of the best path to this vertex. (INFINITY if this node has not been visited.)
		double cost = INFINITY;
		// Whether this vertex has been visited.
		bool visited = false;
	};

	// Initialize a vector of "parent" vertices for each graph vertex.
	std::vector<Parent> parents(graph.getRoadmap().m_vertices.size());

	/**
	 * A candidate edge to explore.
	 */
	struct Candidate {
		// The vertex we're coming from. (nullopt if this is the start vertex.)
		std::optional<PRMCustom::Vertex> to_vertex;
		// The vertex we're going to.
		PRMCustom::Vertex from_vertex;
		// The cost of the edge.
		double cost = INFINITY;
	};

	// Initialize a priority queue of candidates.
	std::priority_queue<Candidate, std::vector<Candidate>, std::function<bool(const Candidate &, const Candidate &)>> queue(
			[](const Candidate &a, const Candidate &b) {
				return a.cost > b.cost;
			});

	// Put the start vertices into the queue.
	for (const auto &start_point: start_points) {
		// Candidate edges incoming to the start vertex have no cost.
		// There is effectively an implicit "start" vertex, signaled by a nullopt "from" vertex.
		queue.push({{}, start_point, 0.0});
	}

	// Iterate over the queue until it's empty.
	while (!queue.empty()) {

		// Get the next vertex to expand.
		Candidate candidate = queue.top();
		// Remove it from the queue.
		queue.pop();

		// Look up the vertex we're entering.
		Parent &parent = parents[candidate.to_vertex];

		// If we've already visited this vertex, then we can skip it.
		if (parent.visited) {
			continue;
		}

		// Mark it as visited.
		parent.visited = true;

		// Update the parent vertex.
		parent.best_parent = candidate.from_vertex;

		// Update the cost.
		parent.cost = candidate.cost;

		// Expand the vertex by looking up its neighbors (boost graph)
		// and adding them to the queue.

		for (const auto& e : boost::make_iterator_range(boost::out_edges(candidate.to_vertex, graph.getRoadmap()))) {

			// Get the target vertex.
			PRMCustom::Vertex target_vertex = boost::target(e, graph);

			// Look up the parent vertex.
			Parent &target_parent = parents[target_vertex];

			// If we've already visited this vertex, then we can skip it.
			if (target_parent.visited) {
				continue;
			}

			// Compute the cost of the edge.
			double edge_cost = graph[e].weight;

			// Add the edge to the queue.
			queue.push({target_vertex, candidate.to_vertex, candidate.cost + edge_cost});

		}

	}

	return nullptr;
}

int main(int argc, char **argv) {

	const int N_TREES = 2;
	const int MAX_FRUIT = 100;

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

		const auto scene = createSceneFromTreeModels(tree_models);

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
			prm.constructRoadmap(ompl::base::timedPlannerTerminationCondition(1.0));

			ompl::base::ScopedState<> shell_state(si);

			std::vector<PRMCustom::Vertex> shell_vertices;

			for (const auto &shell_point: shell_points) {
				shell.stateFromPoint(shell_point, shell_state.get());
				shell_vertices.push_back(prm.insert_state(shell_state.get()));
			}

			// Connect to the goals.
			std::vector<std::vector<PRMCustom::Vertex>> goal_vertices;

			for (const auto &goal_region: goal_regions) {
				// FIXME: Yikes ugly cast.
				prm.tryConnectGoal(*goal_region->as<ompl::base::GoalSampleableRegion>(), 5);
			}

			// Use a variant of Dijkstra to find the shortest path from any of the shell vertices to any of the goal vertices.
			auto paths_in_graph = any_to_any(prm.getRoadmap(), shell_vertices, goal_vertices);

			throw std::runtime_error("Not implemented.");

			return {};

		};

		const std::vector<std::pair<std::string, BatchApproachFn>> planners = {
				{"makeshift", batch_makeshift}
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

			tree_results[planner_name] = planner_results;

		}

		results[tree_models.tree_name] = tree_results;

	}

	std::ofstream file("analysis/data/approach_planning.json");
	file << results;
	file.close();

}