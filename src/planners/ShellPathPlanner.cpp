
#include "ShellPathPlanner.h"
#include "../traveling_salesman.h"
#include "../probe_retreat_move.h"
#include "../DronePathLengthObjective.h"
#include "../experiment_utils.h"
#include <utility>


std::shared_ptr<OMPLSphereShellWrapper<Eigen::Vector3d> > PaddedSphereShellAroundLeavesBuilder::buildShell(const AppleTreePlanningScene &scene_info,
																						 const ompl::base::SpaceInformationPtr &si) const {

	auto enclosing = compute_enclosing_sphere(scene_info.scene_msg, 0.0);

	enclosing.radius += padding * (enclosing.center.z() - enclosing.radius);

	std::cout << "Enclosing sphere: " << enclosing.center << " " << enclosing.radius << std::endl;

	auto shell = std::make_shared<SphereShell>(enclosing.center, enclosing.radius);

	return std::make_shared<OMPLSphereShellWrapper<Eigen::Vector3d> >(shell, si);

}

Json::Value PaddedSphereShellAroundLeavesBuilder::parameters() const {
	Json::Value result;
	result["padding"] = padding;
	return result;
}

PaddedSphereShellAroundLeavesBuilder::PaddedSphereShellAroundLeavesBuilder(double padding) : padding(padding) {
}
