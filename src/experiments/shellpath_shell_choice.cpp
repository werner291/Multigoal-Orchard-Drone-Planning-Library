/**
 * A question was raised about the effects of the size of the sphere.
 *
 * The prediction as that local optimization would cancel any effects that this might have,
 * as long as the sphere was big enough to lie outside of the tree, such that paths were collision-free.
 *
 * This experiment tests that prediction.
 */

#include <thread>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/linear_distribute.hpp>
#include "../run_experiment.h"
#include "../DronePathLengthObjective.h"
#include "../experiment_utils.h"

using namespace std;

int main(int argc, char **argv) {

	/**
	 * We shall "pad" the sphere radius with these amounts.
	 */
	auto paddings = ranges::views::linear_distribute(0.0, 5.0, 11);

	// We'll get a planner-parameter combo for each.
	std::vector<NewMultiGoalPlannerAllocatorFn> planner_allocators = paddings
		| ranges::views::transform([](double padding) -> NewMultiGoalPlannerAllocatorFn {
			return [=](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<MultiGoalPlanner> {

				// We optimize for path length only.
				auto opt = std::make_shared<DronePathLengthObjective>(si);

				// Single-goal planner methods are fairly standard.
				auto ptp = std::make_shared<SingleGoalPlannerMethods>(
						// 1s seems to generally work well as a maximum
						1.0,
						si, 
						opt,
						// PRM star as usual.
						[](auto si) { return std::make_shared<ompl::geometric::PRMstar>(si);}, 
						true, 
						true, 
						true
						);

				// This is the important bit that varies between planners: we use a shell with various degrees of inflation here.
				auto shell_builder = std::make_shared<PaddedSphereShellAroundLeavesBuilder>(padding);

				return std::make_shared<ShellPathPlanner>(
						true, // Yes, we re-pick the shell state
						ptp,
						shell_builder
						);

			};
		}) | ranges::to_vector;

	run_planner_experiment(
			planner_allocators,
			"analysis/shellpath_paddings.json",
			20,
			{150},
			{"appletree"},
			thread::hardware_concurrency(),
			// We remove the ground plane so that inflating the sphere doesn't cause collisions.
			// We're really only interested in whether this affects the path length; that it'd cause
			// collisions is kinda obvious.
			false
			);
}