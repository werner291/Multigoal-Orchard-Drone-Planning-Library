/*
 * This is the experiment entry code for the ICRA 2023 paper.
 *
 * It is known to work correctly on rev 2ef8176 (tagged fullrun-icra2023)
 */

#include <range/v3/range/conversion.hpp>
#include "../run_experiment.h"
#include "MakeshiftPrmApproachPlanningMethods.h"

int main(int argc, char **argv) {

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    std::vector<NewMultiGoalPlannerAllocatorFn> planners;

	// Use known-good parameters here. We don't necessarily need huge diversity since this is our planner.

	for (double ptp_budget : {0.4, 0.5, 1.0}) {
		auto mkptp = [=](const ompl::base::SpaceInformationPtr &si) {

			return std::make_shared<SingleGoalPlannerMethods>(
					ptp_budget,
					si,
					std::make_shared<DronePathLengthObjective>(si),
					[](auto si) { return std::make_shared<ompl::geometric::PRM>(si); },
					true,
					true,
					true
					);
		};

		planners.emplace_back([=](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {


				auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);
				auto shellBuilder = [](const AppleTreePlanningScene &scene_info,
									   const ompl::base::SpaceInformationPtr &si) {

					auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info,
																											  0.1));

					return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);
				};
				return std::make_shared<ShellPathPlanner<Eigen::Vector3d>>(shellBuilder, std::move(approach_methods), true);


		});
	}

	// To prove conclusively that our planner is better than this one, we'll want to test a large number of parameters.
    auto tsp_over_prm_allocators = make_tsp_over_prm_allocators(
			// Samples per goal.
			{2,3,4,5,6,7,8,9},
			// PRM* build-up times.
			{1.0, 2.0, 5.0, 10.0, 15.0, 20.0},
			// Whether to optimize the segments of the full path after the fact.
			{true}
			);

    planners.insert(planners.end(), tsp_over_prm_allocators.begin(), tsp_over_prm_allocators.end());

	run_planner_experiment(planners,
						   "analysis/full_experiment.json",
						   10,
						   {10, 50, 100, 150},
						   {"appletree", "lemontree2", "orangetree4"},
						   std::thread::hardware_concurrency(),
						   true);

    return 0;
}