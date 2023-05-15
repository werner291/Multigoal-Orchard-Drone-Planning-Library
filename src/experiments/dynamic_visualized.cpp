#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../AppleTreePlanningScene.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../planners/ShellPathPlanner.h"
#include "../CurrentPathState.h"

#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/goal_events.h"
#include "../DynamicGoalVisitationEvaluation.h"
#include "../planners/CachingDynamicPlanner.h"
#include "../ORToolsTSPMethods.h"
#include "../utilities/run_experiments.h"

#include "../DynamicGoalsetPlanningProblem.h"
#include "../utilities/MeshOcclusionModel.h"
#include "../utilities/alpha_shape.h"
#include "../planners/ChangeAccumulatingPlannerAdapter.h"
#include "../planners/InitialOrbitPlanner.h"
#include "../planner_allocators.h"
#include "../visualization/visualize_dynamic.h"

#include <vtkProperty.h>
#include <range/v3/view/iota.hpp>

using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

struct Experiment {
	std::pair<std::string, DynamicPlannerAllocatorFn> *planner;
	std::pair<Json::Value, DynamicGoalsetPlanningProblem> *problem;
};

Json::Value toJSON(const Experiment &experiment) {
	Json::Value result;
	result["planner"] = experiment.planner->first;
	result["problem"] = experiment.problem->first;
	return result;
}

Json::Value runDynamicPlannerExperiment(const AppleTreePlanningScene &scene,
										const moveit::core::RobotModelPtr &robot,
										const Experiment &experiment) {// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, scene);

	// Allocate the planner.
	auto ompl_planner = experiment.planner->second(si);

	// Wrap it into the adapter that lets us use it with MoveIt types.
	auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(ompl_planner, si, ss);

	// Create the evaluation object.
	DynamicGoalVisitationEvaluation eval(adapter,
										 experiment.problem->second.start_state,
										 scene,
										 experiment.problem->second.apple_discoverability,
										 *experiment.problem->second.can_see_apple);

	// Record the start time.
	auto start_time = std::chrono::high_resolution_clock::now();

	// Run the planner for the initial set of goals.
	eval.computeNextTrajectory();

	// Run the planner until it has no more goals to visit and returns nullopt.
	while (eval.getUpcomingGoalEvent().has_value()) {
		if (!eval.computeNextTrajectory().has_value()) {
			break;
		}
	}

	// Record the end time.
	auto end_time = std::chrono::high_resolution_clock::now();

	int n_visited = (int) ranges::count_if(eval.getDiscoveryStatus(),
										   [](const auto &status) { return status == utilities::VISITED; });

	double total_path_length = ranges::accumulate(
			eval.getSolutionPathSegments() | ranges::views::transform([](const auto &segment) {
				return segment.path.length();
			}), 0.0);

	std::chrono::nanoseconds total_time = ranges::accumulate(
			eval.getSolutionPathSegments() | ranges::views::transform([](const auto &segment) {
				return segment.time;
			}), std::chrono::nanoseconds(0));

	Json::Value result;
	result["n_visited"] = n_visited;
	result["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(total_time).count();
	result["total_path_length"] = total_path_length;

	return result;
}

int main(int argc, char **argv) {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");

	// Convert the meshes to a planning scene message.
	const auto scene = AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(
			treeMeshesToMoveitSceneMsg(meshes))), .apples = meshes.fruit_meshes |
															ranges::views::transform(appleFromMesh) |
															ranges::to_vector};

	// Load the robot model.
	const auto robot = loadRobotModel();

	using namespace ranges;

	utilities::CanSeeAppleFn distance_occlusion = [](const moveit::core::RobotState &state, const Apple &apple) {

		Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

		const double discovery_max_distance = 1.0;

		return (ee_pos - apple.center).squaredNorm() < discovery_max_distance * discovery_max_distance;

	};

	auto alphashape = alphaShape(meshes.leaves_mesh.vertices | ranges::views::transform([](const auto &v) {
		return Eigen::Vector3d{v.x, v.y, v.z};
	}) | ranges::to_vector, LEAVES_ALPHA_SQRTRADIUS);

	MeshOcclusionModel occlusion_model(alphashape, 0);

	utilities::CanSeeAppleFn leaf_alpha_occlusion = [&](const moveit::core::RobotState &state, const Apple &apple) {

		Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

		return !occlusion_model.checkOcclusion(apple.center, ee_pos);

	};

	Proportions probs{.fraction_true_given = 1.0, .fraction_false_given = 0.0, .fraction_discoverable = 0.0,};

	const auto start_state = randomStateOutsideTree(robot, 0);
	auto apple_discoverability = generateAppleDiscoverability(probs, 42, scene.apples.size());

	std::cout << "Starting planning with " << apple_discoverability.size() << " apples in total, of which "
			  << ranges::count(apple_discoverability, DISCOVERABLE) << " are discoverable." << std::endl;

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, scene);

	auto planner = static_planner(si); //dynamic_planner_initial_orbit(si);

	auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(planner, si, ss);

	DynamicGoalVisitationEvaluation eval(adapter, start_state, scene, apple_discoverability, leaf_alpha_occlusion);

	visualizeEvaluation(meshes, scene, robot, start_state, apple_discoverability, eval);

}


