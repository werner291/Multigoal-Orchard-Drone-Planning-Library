#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <fstream>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "../utilities/experiment_utils.h"
#include "../GreatCircleMetric.h"
#include "../probe_retreat_move.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"
#include "../PathLengthPredictor.h"

int main(int argc, char **argv) {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");
	auto drone = loadRobotModel();

	// Convert the meshes to a planning scene message.
	const auto scene = AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(
			treeMeshesToMoveitSceneMsg(meshes))), .apples = meshes.fruit_meshes |
															ranges::views::transform(appleFromMesh) |
															ranges::to_vector};

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(drone);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, scene);

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	Json::Value v;
	v["name"] = "DendriticConvexHullDistancePredictor";

	Json::Value v2;
	v2["name"] = "DendriticConvexHullDistancePredictor";



	std::vector<std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>>> predictors {
		pairWithJson(EuclideanDistancePredictor()),
		pairWithJson(GreatCircleDistancePredictor::mec_around_leaves(scene)),
		pairWithJson(CuttingPlaneConvexHullDistancePredictor::around_leaves(scene)),
		{v, std::make_shared<CGALConvexHullDistancePredictor>(meshes.leaves_mesh)},
		{v2, std::make_shared<DendriticConvexHullDistancePredictor>(meshes.leaves_mesh)}
	};

	auto approach_planner = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

	Json::Value results;

	for (size_t i = 0; i < 500; ++i) {

		// Pick two random apples.

		std::random_device rd;
		std::mt19937 rng(rd());

		size_t apple_i = std::uniform_int_distribution<size_t>(0, scene.apples.size() - 1)(rng);
		size_t apple_j = std::uniform_int_distribution<size_t>(0, scene.apples.size() - 2)(rng);
		if (apple_j >= apple_i) {
			apple_j++;
		}

		ompl::base::GoalPtr goal_i = std::make_shared<DroneEndEffectorNearTarget>(si,
																				  0.05,
																				  scene.apples[apple_i].center);
		ompl::base::GoalPtr goal_j = std::make_shared<DroneEndEffectorNearTarget>(si,
																				  0.05,
																				  scene.apples[apple_j].center);

		Json::Value result;

		for (const auto &[json, predictor] : predictors) {
			result["predictor"] = json["name"];
			result["predicted_length"] = predictor->predict_path_length(scene.apples[apple_i], scene.apples[apple_j]);
		}


		// Plan approaches to the apples.
		auto approach_i = approach_planner->approach_path(goal_i, *shell);
		auto approach_j = approach_planner->approach_path(goal_j, *shell);

		// Skip if these fail.
		if (!approach_i || !approach_j) {
			continue;
		}



		// Plan a path between the two apples.
		ompl::geometric::PathGeometric path(si);
		path.append(approach_i->robot_path);
		path.reverse();
		path.append(shell->shellPath(approach_i->shell_point, approach_j->shell_point));
		path.append(approach_j->robot_path);

		result["actual_length_unoptimized"] = path.length();

		// Optimize the path.
		result["actual_length"] = optimize(path, std::make_shared<DronePathLengthObjective>(si), si).length();

		std::cout << "Completed run " << i << std::endl;

		results.append(result);

	}

	std::ofstream ofs;
	ofs.open("analysis/data/path_length_prediction.json");
	ofs << results;
	ofs.close();

}