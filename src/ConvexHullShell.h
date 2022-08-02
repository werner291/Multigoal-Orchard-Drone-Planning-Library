#ifndef NEW_PLANNERS_CONVEXHULLSHELL_H
#define NEW_PLANNERS_CONVEXHULLSHELL_H

#include "SphereShell.h"
#include "planners/ShellPathPlanner.h"
#include <ompl/datastructures/NearestNeighborsGNAT.h>

struct ConvexHullPoint {
	size_t face_id;
	Eigen::Vector3d position;
};

class ConvexHullShellBuilder : public ShellPathPlanner<ConvexHullPoint>::ShellBuilder {

public:
	std::shared_ptr<OMPLSphereShellWrapper<ConvexHullPoint>>
	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) const override;

	[[nodiscard]] Json::Value parameters() const override;
};

class ConvexHullShell : public CollisionFreeShell<ConvexHullPoint> {

	std::vector<Eigen::Vector3d> vertices;

	struct Facet {
		size_t a, b, c;
		size_t neighbour_ab, neighbour_bc, neighbour_ca;
	};

	std::vector<Facet> facets;

	struct NNGNATEntry {
		bool operator==(const NNGNATEntry &rhs) const;

		bool operator!=(const NNGNATEntry &rhs) const;

		size_t face_index;
		Eigen::Vector3d at;

	};

	ompl::NearestNeighborsGNAT<NNGNATEntry> facet_index;

	size_t guess_closest_face(const Eigen::Vector3d &a) const;

public:

	ConvexHullShell(const shape_msgs::msg::Mesh &mesh);

	[[nodiscard]] moveit::core::RobotState
	state_on_shell(const moveit::core::RobotModelConstPtr &drone, const ConvexHullPoint &a) const override;

	[[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																	  const ConvexHullPoint &a,
																	  const ConvexHullPoint &b) const override;

	[[nodiscard]] double predict_path_length(const ConvexHullPoint &a, const ConvexHullPoint &b) const override;

	ConvexHullPoint gaussian_sample_near_point(const ConvexHullPoint &near) const override;

	ConvexHullPoint project(const moveit::core::RobotState &st) const override;

	ConvexHullPoint project(const Apple &st) const override;

protected:
	[[nodiscard]] ConvexHullPoint project(const Eigen::Vector3d &a) const;

	void match_faces();

	void init_gnat();


	std::vector<ConvexHullPoint> convex_hull_walk(const ConvexHullPoint& a, const ConvexHullPoint& b) const;
};

moveit::core::RobotState robotStateFromFacing(const moveit::core::RobotModelConstPtr &drone,
											  const Eigen::Vector3d &desired_ee_pos,
											  const Eigen::Vector3d &required_facing);


#endif //NEW_PLANNERS_CONVEXHULLSHELL_H
