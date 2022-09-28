/**
 * exp:sphere_shell_arm_angle
 */

#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/linear_distribute.hpp>
#include "../run_experiment.h"

using namespace std;

int main(int argc, char **argv) {

	auto alloc_opt = [](const ompl::base::SpaceInformationPtr &si) {
		return make_shared<DronePathLengthObjective>(si);
	};

	auto alloc_ptp = [=](const ompl::base::SpaceInformationPtr &si) {
		// Single-goal planner methods are fairly standard.
		return make_shared<SingleGoalPlannerMethods>(
				// 1s seems to generally work well as a maximum
				1.0, si, alloc_opt(si),
				// PRM star as usual.
				[](auto si) { return make_shared<ompl::geometric::PRMstar>(si); },
				true,
				true,
				true);
	};

	// We'll get a planner-parameter combo for each.
	vector<NewMultiGoalPlannerAllocatorFn> planner_allocators = {

			[=](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) -> shared_ptr<MultiGoalPlanner> {
				auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);
				auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr& si) {
					return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(paddedSphericalShellAroundLeaves(scene_info, 0.1), si);
				};
				return make_shared<ShellPathPlanner<Eigen::Vector3d>>(shellBuilder,std::move(approach_methods),true);
			},

			[=](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) -> shared_ptr<MultiGoalPlanner> {

				auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);
				auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr& si) {

					auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

					return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);
				};
				return make_shared<ShellPathPlanner<Eigen::Vector3d>>(shellBuilder,std::move(approach_methods),true);
			},

	};

	run_planner_experiment(planner_allocators,
						   "analysis/shellpath_shell_comparison.json",
						   10,
						   {150},//{50, 150},
						   {"appletree"},//, "lemontree2", "orangetree4"},
						   thread::hardware_concurrency(),
						   true);
}