#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <fstream>
#include <range/v3/range/conversion.hpp>
#include "../utilities/experiment_utils.h"
#include "../GreatCircleMetric.h"
#include "../probe_retreat_move.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"
#include "../PathLengthPredictor.h"

template<typename ShellPoint>
Json::Value compute_goaltogoal_stats(ompl::base::GoalPtr goal_i,
									 ompl::base::GoalPtr goal_j,
									 const OmplShellSpace<ShellPoint> &shell,
									 const ompl::base::SpaceInformationPtr &si) {

	auto approach_planner = std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si);

	// Plan approaches to the apples.
	auto approach_i = approach_planner->approach_path(goal_i, shell);
	auto approach_j = approach_planner->approach_path(goal_j, shell);

	// Skip if these fail.
	if (!approach_i || !approach_j) {
		return "Approach Planning failed.";
	}

	// Plan a path between the two apples.
	ompl::geometric::PathGeometric path(si);
	path.append(approach_i->robot_path);
	path.reverse();
	auto shellPath = shell.shellPath(approach_i->shell_point, approach_j->shell_point);
	path.append(shellPath);
	path.append(approach_j->robot_path);

	Json::Value result;

	result["actual_length_unoptimized"] = path.length();
	result["shell_path_length"] = shellPath.length();

	// Optimize the path.
	result["actual_length"] = optimize(path, std::make_shared<DronePathLengthObjective>(si), si).length();

	return result;

}


int main(int argc, char **argv) {

	const auto drone = loadRobotModel();

	// Load the apple tree meshes.

	const auto scenes = scenes_for_trees({"appletree", "lemontree2", "orangetree4", "orangetree2"}, 500);

	Json::Value results_by_scene;

	for (auto &scene: scenes) {

		Json::Value scene_results;

		// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
		// So, we just re-create the state space every time just to be safe.
		auto ss = omplStateSpaceForDrone(drone, TRANSLATION_BOUND);

		// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
		// we'll need to copy this for every thread
		auto si = loadSpaceInformation(ss, scene);

		auto sphere_shell = OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1)), si);
		auto chull_shell = OmplShellSpace<ConvexHullPoint>::fromWorkspaceShell(cuttingPlaneConvexHullAroundLeaves(scene, 0.0, 0.0), si);
		auto cgal_shell = OmplShellSpace<mgodpl::cgal_utils::CGALMeshPointAndNormal>::fromWorkspaceShell(convexHullAroundLeavesCGAL(scene, 0.0, 0.0), si);

		std::vector<std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>>> predictors {
				pairWithJson(EuclideanDistancePredictor()),
				pairWithJson(GreatCircleDistancePredictor::mec_around_leaves(scene, false)),
				pairWithJson(GreatCircleDistancePredictor::mec_around_leaves(scene, true)),
				pairWithJson(HelicalDistancePredictor::around_leaves(scene, false)),
				pairWithJson(HelicalDistancePredictor::around_leaves(scene, true)),
				pairWithJson(CuttingPlaneConvexHullDistancePredictor::around_leaves(scene, false)),
				pairWithJson(CuttingPlaneConvexHullDistancePredictor::around_leaves(scene, true)),
				pairWithJson(std::make_shared<CGALConvexHullDistancePredictor>(scene, false)),
				pairWithJson(std::make_shared<CGALConvexHullDistancePredictor>(scene, true)),
				pairWithJson(std::make_shared<DendriticConvexHullDistancePredictor>(scene)),
		};

		for (size_t i = 0; i < 200; ++i) {

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

			for (const auto &[json, predictor]: predictors) {

				Json::Value prediction;
				prediction["predicted_length"] = predictor->predict_path_length(scene.apples[apple_i],
																			   scene.apples[apple_j]);
				prediction["predictor_parameters"] = json;

				result["predictions"].append(prediction);
			}

			result["on_sphere"] = compute_goaltogoal_stats(goal_i, goal_j, *sphere_shell, si);
			result["on_chull"] = compute_goaltogoal_stats(goal_i, goal_j, *chull_shell, si);
			result["on_cgal_chull"] = compute_goaltogoal_stats(goal_i, goal_j, *cgal_shell, si);

			std::cout << "Completed run " << i << std::endl;

			results_by_scene[scene.scene_msg->name].append(result);

		}

	}

	std::ofstream ofs;
	ofs.open("analysis/data/path_length_prediction.json");
	ofs << results_by_scene;
	ofs.close();

}