#include <thread>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/transform.hpp>
#include "../run_experiment.h"
#include "../DronePathLengthObjective.h"
#include "../experiment_utils.h"

using namespace std;

class RatioOfClearancePaddingShellGenerator : public ShellPathPlanner::ShellBuilder {

	double padding_ratio;
public:
	explicit RatioOfClearancePaddingShellGenerator(double paddingRatio) : padding_ratio(paddingRatio) {
	}

public:
	shared_ptr<OMPLSphereShellWrapper> buildShell(const AppleTreePlanningScene &scene_info,
												  const ompl::base::SpaceInformationPtr &si) override {

		auto enclosing = compute_enclosing_sphere(scene_info.scene_msg, 0.0);

		enclosing.radius += padding_ratio * (enclosing.center.z() - enclosing.radius);

		std::cout << "Enclosing sphere: " << enclosing.center << " " << enclosing.radius << std::endl;

		return std::make_shared<OMPLSphereShellWrapper>(std::make_shared<SphereShell>(enclosing.center, enclosing.radius), si);

	}

	[[nodiscard]] Json::Value parameters() const override {
		Json::Value params;
		params["padding_ratio"] = padding_ratio;
		return params;
	}
};

int main(int argc, char **argv) {

	double paddings[] = {
			0.1, 0.2, 0.5, 0.9
	};

	std::vector<NewMultiGoalPlannerAllocatorFn> planner_allocators = paddings
		| ranges::views::transform([](double padding) -> NewMultiGoalPlannerAllocatorFn {
			return [&](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<MultiGoalPlanner> {

				auto opt = std::make_shared<DronePathLengthObjective>(si);

				auto ptp = std::make_shared<SingleGoalPlannerMethods>(1.0, si, opt, [](auto si) {
					return std::make_shared<ompl::geometric::PRMstar>(si);
				}, true, true, true);

				return std::make_shared<ShellPathPlanner>(true, ptp, std::make_shared<RatioOfClearancePaddingShellGenerator>(padding));

			};
		}) | ranges::to_vector;

	run_planner_experiment(planner_allocators,
						   "analysis/shellpath_paddings.json",
						   10,
						   {150},//{10, 50, 100, 150},
						   {"appletree"},
						   thread::hardware_concurrency());
}