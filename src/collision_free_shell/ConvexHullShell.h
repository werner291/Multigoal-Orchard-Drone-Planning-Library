#ifndef NEW_PLANNERS_CONVEXHULLSHELL_H
#define NEW_PLANNERS_CONVEXHULLSHELL_H

#include "SphereShell.h"
#include "../planners/ShellPathPlanner.h"
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/view/iota.hpp>

/**
 * A reference to some location on a ConvexHullShell.
 */
struct ConvexHullPoint {
	/// The facet identifier of the facet containing this point.
	size_t face_id;
	/// The position of this point in 3D Euclidean space.
	Eigen::Vector3d position;
};

/**
 * A ShellBuilder that builds a ConvexHullShell around the leaf vertices in a tree.
 */
class ConvexHullShellBuilder : public ShellPathPlanner<ConvexHullPoint>::ShellBuilder {

public:
	ConvexHullShellBuilder() = default;

	[[nodiscard]] std::shared_ptr<OMPLSphereShellWrapper<ConvexHullPoint>>
	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) const override;

	[[nodiscard]] Json::Value parameters() const override;
};

class ConvexHullShell : public CollisionFreeShell<ConvexHullPoint> {

	/// A set of all the vertices in the convex hull.
	std::vector<Eigen::Vector3d> vertices;

	/// A padding distance to add to shell states.
	double padding = 0.1;

	/// The Facet structure that contains references to the vertices and neighbouring facets.
	struct Facet {
		/// The vertices that make up this facet.
		size_t a, b, c;
		/// The neighbouring facets, by the edge that connects to them.
		size_t neighbour_ab, neighbour_bc, neighbour_ca;

		[[nodiscard]] std::array<size_t, 3> neighbours() const;
	};

	/// The facets that make up the convex hull.
	std::vector<Facet> facets;

	/// The entry type for the nearest-neighbour structure.
	struct NNGNATEntry {
		bool operator==(const NNGNATEntry &rhs) const;

		bool operator!=(const NNGNATEntry &rhs) const;

		size_t face_index;
		Eigen::Vector3d at;

	};

	/// A GNAT that gives a indexes all facets by their center of gravity.
	ompl::NearestNeighborsGNAT<NNGNATEntry> facet_index;

	/**
	 * Find the facet whose center of gravity is closest to the given point,
	 * used as an initial guess for locating the facet closest to the point.
	 *
	 * This operation takes O(log n) time in the number of facets.
	 *
	 * @param a 	The point to find the facet closest to.
	 * @return 		The index of the facet whose COG is closest to the point.
	 */
	size_t guess_closest_face(const Eigen::Vector3d &a) const;

public:

	/**
	 * Construct a ConvexHullShell from a convex hull mesh.
	 *
	 * Warning: The mesh given is assumed to already be convex. If it is not,
	 * you may build one using the convexHull function.
	 *
	 * @param mesh 		The mesh to use. (Note: assumed to be convex!)
	 */
	ConvexHullShell(const shape_msgs::msg::Mesh &mesh);

	/**
	 * Construct a RobotState whose end effector is at the given point, plus a padding distance.
	 *
	 * @param drone 	The robot model to use.
	 * @param a 		The point to use.
	 * @return 			The constructed RobotState.
	 */
	[[nodiscard]] moveit::core::RobotState
	state_on_shell(const moveit::core::RobotModelConstPtr &drone, const ConvexHullPoint &a) const override;

	/**
	 * Plan a path of RobotStates from the given start to the given goal.
	 *
	 * Linear interpolation between the states in the returned vector are guaranteed to have the robot's end-effector
	 * on the convex hull at all times (+ padding), and the path is guaranteed to be collision-free.
	 *
	 * @param drone 	The robot model to use.
	 * @param start 	The start point.
	 * @param goal 		The goal point.
	 *
	 * @return 			A vector of RobotStates, which serve as anchor points for linear interpolation.
	 */
	[[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																	  const ConvexHullPoint &a,
																	  const ConvexHullPoint &b) const override;

	/**
	 * Predict the length of a path from the given start to the given goal by generating and measuring the length
	 * of a segmented line across the surface without constructing all the robot states.
	 *
	 * @param a 	The start point.
	 * @param b 	The goal point.
	 * @return 		The predicted length of the path.
	 */
	[[nodiscard]] double predict_path_length(const ConvexHullPoint &a, const ConvexHullPoint &b) const override;

	/**
	 * Generate a ConvexHullPoint in a (roughly) gaussian distribution around another point.
	 *
	 * @param near 	The point to generate around.
	 * @return 		The generated point.
	 */
	ConvexHullPoint gaussian_sample_near_point(const ConvexHullPoint &near) const override;

	/**
	 * Find the ConvexHullPoint of the projection of the end-effector of the robot onto the convex hull.
	 *
	 * @param st 		The RobotState to use.
	 * @return 			The projected point.
	 */
	ConvexHullPoint project(const moveit::core::RobotState &st) const override;

	/**
	 * Find the ConvexHullPoint of the projection of an apple onto the convex hull.
	 *
	 * @param st 		The RobotState to use.
	 * @return 			The projected point.
	 */
	ConvexHullPoint project(const Apple &st) const override;

	/**
	 * The number of facets in the convex hull.
	 *
	 * Valid facet indices will be in the range [0, num_facets).
	 *
	 * @return 		The number of facets.
	 */
	size_t num_facets() const;

	/**
	 * Get the facet structure for a given index.
	 *
	 * @param i 	The index of the facet (0 <= i < num_facets()).
	 * @return 		The facet structure.`
	 */
	const Facet& facet(size_t i) const;

	/**
	 * Compute the outward-pointing normal of a facet. Normalization is applied.
	 *
	 * @param i 	The index of the facet (0 <= i < num_facets()).
	 * @return 		The normal (unit length).
	 */
	Eigen::Vector3d facet_normal(size_t i) const;

	Eigen::Hyperplane<double, 3> facet_plane(size_t i) const;

	/**
	 * Look up a vertex of the convex hull.
	 *
	 * @param i 	The index of the vertex (0 <= i < num_vertices()).
	 * @return 		The vertex.
	 */
	const Eigen::Vector3d& vertex(size_t i) const;

	/**
	 * Look up the three vertices of a facet (in Euclidean space).
	 *
	 * @param facet_i 	The index of the facet (0 <= facet_i < num_facets()).
	 * @return 			The three vertices.
	 */
	std::array<Eigen::Vector3d, 3> facet_vertices(size_t facet_i) const;

	/**
	 * Look up the signed distance of a point to the convex hull.
	 *
	 * A distance >0 indicates the point is outside the convex hull, <0 indicates it is inside.
	 *
	 * Note: this operation is O(n) in the number of facets.
	 *
	 * @param pt 	The point to check.
	 * @return 		The signed distance.
	 */
	double signed_distance(const Eigen::Vector3d &pt) const;

	/**
	 * Generate a path of ConversHullPoints from the given start to the given goal across the surface of the convex hull.
	 *
	 * At every edge traversal, a point will be generated for the facet being exited, and a point for the facet being entered,
	 * so note that the returned path will contain duplicate points in Euclidean terms.
	 *
	 * @param a	The start point.
	 * @param b	The goal point.
	 * @return	A vector of ConvexHullPoints.
	 */
	std::vector<ConvexHullPoint> convex_hull_walk(const ConvexHullPoint& a, const ConvexHullPoint& b) const;

protected:

	/**
	 * Project a point onto the convex hull, returning a ConvexHullPoint (that includes the facet index).
	 *
	 * @param a 	The point to project.
	 * @return 		The projected point.
	 */
	[[nodiscard]] ConvexHullPoint project(const Eigen::Vector3d &a) const;

	/**
	 * Internal: populate the neighbour_ab, neighbour_bc, neighbour_ca fields of each facet.
	 */
	void match_faces();

	/**
	 * Internal: Build the facet lookup structure.
	 */
	void init_gnat();

	/**
	 *
	 * Find the signed distance of a point to the plane extending a facet of the convex hull.
	 *
	 * @param ptr 			The point to check.
	 * @param facet_index 	The index of the facet.
	 * @return 				The signed distance.
	 */
	double facet_signed_distance(const Eigen::Vector3d ptr, size_t facet_index) const;
};

#endif //NEW_PLANNERS_CONVEXHULLSHELL_H
