#include <thread>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/transform.hpp>
#include "../run_experiment.h"
#include "../planners/ShellPathPlanner.h"
#include "../DronePathLengthObjective.h"
#include "../experiment_utils.h"

using namespace std;

int main(int argc, char **argv) {

	double paddings[] = {
			0.1, 0.2, 0.5, 0.9
	};

	std::vector<NewMultiGoalPlannerAllocatorFn> planner_allocators = paddings
		| ranges::views::transform([](double padding) -> NewMultiGoalPlannerAllocatorFn {
			return [&](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<MultiGoalPlanner> {

				auto opt = std::make_shared<DronePathLengthObjective>(si);

				auto make_shell_fn = [&](const AppleTreePlanningScene &scene) {

					auto enclosing = compute_enclosing_sphere(scene.scene_msg, 0.0);

					enclosing.radius += padding * (enclosing.center.z() - enclosing.radius);

					std::cout << "Enclosing sphere: " << enclosing.center << " " << enclosing.radius << std::endl;

					return std::make_shared<SphereShell>(enclosing.center, enclosing.radius);

				};

				auto ptp = std::make_shared<SingleGoalPlannerMethods>(1.0, si, opt, [](auto si) {
					return std::make_shared<ompl::geometric::PRMstar>(si);
				}, true, true, true);

				return static_pointer_cast<MultiGoalPlanner>(std::make_shared<ShellPathPlanner>(true, ptp, make_shell_fn));

			};
		}) | ranges::to_vector;

	run_planner_experiment(planner_allocators,
						   "analysis/shellpath_paddings.json",
						   10,
						   {150},//{10, 50, 100, 150},
						   thread::hardware_concurrency());
}