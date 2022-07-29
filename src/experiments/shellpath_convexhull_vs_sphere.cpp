/**
 * How effective using a convex hull as a collision free shell versus a spherical shell?
 */

#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/linear_distribute.hpp>
#include <range/v3/view/transform.hpp>
#include "../run_experiment.h"
#include "../DronePathLengthObjective.h"
#include "../ConvexHullShell.h"

using namespace std;

int main(int argc, char **argv) {

	auto alloc_opt = [](const ompl::base::SpaceInformationPtr &si) {
		return std::make_shared<DronePathLengthObjective>(si);
	};

	auto alloc_ptp = [=](const ompl::base::SpaceInformationPtr &si) {
		// Single-goal planner methods are fairly standard.
		return std::make_shared<SingleGoalPlannerMethods>(
				// 1s seems to generally work well as a maximum
				1.0, si, alloc_opt(si),
				// PRM star as usual.
				[](auto si) { return std::make_shared<ompl::geometric::PRMstar>(si); },
				true,
				true,
				true);
	};

	using namespace std;

	vector<shared_ptr<const ShellPathPlanner::ShellBuilder>> shell_builder{
		make_shared<ConvexHullShellBuilder>(),
//		make_shared<PaddedSphereShellAroundLeavesBuilder>(0.1)
	};

	// We'll get a planner-parameter combo for each.
	std::vector<NewMultiGoalPlannerAllocatorFn> planner_allocators =
			shell_builder | ranges::views::transform([=](auto shell_builder) -> NewMultiGoalPlannerAllocatorFn {
				return [=](const AppleTreePlanningScene &scene_info,
						   const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<MultiGoalPlanner> {
					return std::make_shared<ShellPathPlanner>(true, alloc_ptp(si), shell_builder);
				};
			}) | ranges::to_vector;

	run_planner_experiment(planner_allocators,
						   "analysis/shellpath_chull_vs_sphere.json",
						   20,
						   {150},
						   {"appletree"},
						   1,//thread::hardware_concurrency(),
						   true);
}