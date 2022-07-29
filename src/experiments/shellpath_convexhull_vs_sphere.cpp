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
				return make_shared<ShellPathPlanner<ConvexHullPoint>>(
						true,
						alloc_ptp(si),
						make_shared<ConvexHullShellBuilder>()
						        );
			}
	};



	run_planner_experiment(planner_allocators,
						   "analysis/shellpath_chull_vs_sphere.json",
						   20,
						   {150},
						   {"appletree"},
						   1,//thread::hardware_concurrency(),
						   true);
}