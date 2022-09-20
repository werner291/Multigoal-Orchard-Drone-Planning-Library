/**
 * This experiment is about the impact of the quality of the centering on the length of the path.
 *
 * The experiment proceeds by using the minimum enclosing sphere to center the sphere, and then
 * applying a vertical offset to the center of the sphere, while increasing the radius of the sphere
 * by the same amount to ensure collision-freeness.
 */

#include <thread>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/linear_distribute.hpp>
#include "../run_experiment.h"
#include "../DronePathLengthObjective.h"
#include "../utilities/experiment_utils.h"

using namespace std;

/**
 * Builds a SphereShell with a configurable vertical bias.
 */
class OffsetSphereBuilder: public ShellPathPlanner::ShellBuilder {
	double offset;
public:
	OffsetSphereBuilder(double offset) : offset(offset) {
	}

public:
	shared_ptr<OMPLShellSpaceWrapper>
	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) override {

		auto enclosing = compute_enclosing_sphere(scene_info.scene_msg, 0.0);

		enclosing.radius += 0.1;

		// By increasing the Z-value of the center, but also increasing the radius at the same time,
		// we keep the guarantee that it contains all leaves.
		enclosing.center.z() += offset;
		enclosing.radius += offset;

		std::cout << "Enclosing sphere: " << enclosing.center << " " << enclosing.radius << std::endl;

		auto shell = std::make_shared<SphereShell>(enclosing.center, enclosing.radius);

		return std::make_shared<OMPLShellSpaceWrapper>(shell, si);

	}

	[[nodiscard]] Json::Value parameters() const override {
		Json::Value params;
		params["offset"] = offset;
		return params;
	}
};

int main(int argc, char **argv) {

	/**
	 * We shall offset the sphere center by these amounts
	 */
	auto offsets = ranges::views::linear_distribute(0.0, 5.0, 11);

	// We'll get a planner-parameter combo for each.
	std::vector<NewMultiGoalPlannerAllocatorFn> planner_allocators = offsets
		| ranges::views::transform([](double offset) -> NewMultiGoalPlannerAllocatorFn {
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
				auto shell_builder = std::make_shared<OffsetSphereBuilder>(offset);

				return std::make_shared<ShellPathPlanner>(
						true, // Yes, we re-pick the shell state
						ptp,
						shell_builder
						);

			};
		}) | ranges::to_vector;

	run_planner_experiment(
			planner_allocators,
			"analysis/shellpath_offsets.json",
			10,
			{150},
			{"appletree"},
			thread::hardware_concurrency(),
			// We remove the ground plane so that inflating the sphere doesn't cause collisions.
			// We're really only interested in whether this affects the path length; that it'd cause
			// collisions is kinda obvious.
			false
			);
}