
#ifndef NEW_PLANNERS_CONVEXHULLSHELL_H
#define NEW_PLANNERS_CONVEXHULLSHELL_H

#include "SphereShell.h"
#include "planners/ShellPathPlanner.h"
#include <ompl/datastructures/NearestNeighborsGNAT.h>

class ConvexHullShellBuilder : public ShellPathPlanner::ShellBuilder {

public:
	std::shared_ptr<OMPLSphereShellWrapper>
	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) override;

	Json::Value parameters() const override;
};

class ConvexHullShell : public CollisionFreeShell {

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

	moveit::core::RobotState
		state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const override;

	std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone,
														const Eigen::Vector3d &a,
														const Eigen::Vector3d &b) const override;

	double predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;

protected:
	Eigen::Vector3d project(const Eigen::Vector3d &a) const override;

	void match_faces();

	void init_gnat();
};


#endif //NEW_PLANNERS_CONVEXHULLSHELL_H
