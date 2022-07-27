
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

size_t ConvexHullShell::closest_vertex(const Eigen::Vector3d &a) {

	/*
	 * Lemma:
	 *
	 * 		Lemma: On a convex hull, the closest point to a point `p`
	 * 		lies in a face connected to the closest mesh vertex `v` to `a`.
	 *
	 * Proof:
	 *
	 *	 	Suppose that p does not lie in a face that has `a` as a vertex.
	 *
	 * 		Consider the triangle `T` with vertices `a,v,p`.
	 * 		Then, consider the geodesic G between `v,p` on M.
	 *
	 *	 	Since `a` and `v` are vertices of distinct faces,
	 *	 	the geodesic must cross at least one edge of the mesh,
	 *	 	let's call this point x.
	 *
	 *	 	Since M is convex, `x` must lie in the interior of `T`.
	 *
	 *	 	Hence, `x` is a point on `M` closer to `a` than `p`.
	 *	 	This contradicts our assumption, and proves the lemma.
	 *
	 */



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
