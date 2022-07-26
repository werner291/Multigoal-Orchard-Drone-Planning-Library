
#include "ConvexHullShell.h"

#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/RboxPoints.h>

moveit::core::RobotState
ConvexHullShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const {
	return moveit::core::RobotState(std::shared_ptr());
}

std::vector<moveit::core::RobotState> ConvexHullShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																	 const Eigen::Vector3d &a,
																	 const Eigen::Vector3d &b) const {
	return std::vector<moveit::core::RobotState>();
}

double ConvexHullShell::predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {
	return 0;
}

Eigen::Vector3d ConvexHullShell::project(const Eigen::Vector3d &a) const {
	return Eigen::Vector3d();
}

std::shared_ptr<OMPLSphereShellWrapper> ConvexHullShellBuilder::buildShell(
		const AppleTreePlanningScene &scene_info,
		const ompl::base::SpaceInformationPtr &si) {

	orgQhull::RboxPoints points;

	for (const auto& col : scene_info.scene_msg.world.collision_objects) {
		if (col.id == "leaves") {
			for (const auto& mesh : col.meshes) {
				for (auto v : mesh.vertices) {
					double coords[3] = {v.x, v.y, v.z};

					orgQhull::QhullPoint p(3, coords);

					points.append(p);
				}
			}
		}
	}

	orgQhull::Qhull qhull;
	qhull.runQhull(points, "");

	std::cout << qhull.facetList() << std::endl;

}

Json::Value ConvexHullShellBuilder::parameters() const {
	return Json::Value();
}
