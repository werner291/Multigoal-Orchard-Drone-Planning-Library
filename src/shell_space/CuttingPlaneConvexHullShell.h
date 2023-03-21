#ifndef NEW_PLANNERS_CUTTINGPLANECONVEXHULLSHELL_H
#define NEW_PLANNERS_CUTTINGPLANECONVEXHULLSHELL_H

#include "SphereShell.h"
#include "../planners/ShellPathPlanner.h"
#include "../utilities/math_utils.h"
#include "../utilities/convex_hull.h"
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/view/iota.hpp>

/**
 * A reference to some location on a CuttingPlaneConvexHullShell.
 */
struct ConvexHullPoint {
	/// The facet identifier of the facet containing this point.
	size_t face_id;
	/// The position of this point in 3D Euclidean space.
	Eigen::Vector3d position;
};
//
///**
// * A ShellBuilder that builds a CuttingPlaneConvexHullShell around the leaf vertices in a tree.
// */
//class ConvexHullShellBuilder : public OmplShellConstructionMethod<ConvexHullPoint> {
//
//	double rotation_weight = 1.0;
//	double padding = 0.1;
//
//	public:
//	ConvexHullShellBuilder(double padding, double rotationWeight);
//
//
//	[[nodiscard]] std::shared_ptr<OMPLShellSpaceWrapper<ConvexHullPoint>>
//	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) const override;
//
//	[[nodiscard]] Json::Value parameters() const override;
//}

class CuttingPlaneConvexHullShell : public WorkspaceShell<ConvexHullPoint> {

	/// A set of all the vertices in the convex hull.
	std::vector<Eigen::Vector3d> vertices;

	/// A padding distance to add to shell states.
	double padding = 0.1;

	double rotation_weight = 1.0;

	/// The Facet structure that contains references to the vertices and neighbouring facets.
	struct Facet {
		/// The vertices that make up this facet.
		size_t a, b, c;
		/// The neighbouring facets, by the edge that connects to them.
		size_t neighbour_ab, neighbour_bc, neighbour_ca;

		[[nodiscard]] std::array<size_t, 3> neighbours() const;

		[[nodiscard]] size_t neighbour(TriangleEdgeId edge_id) const;

		[[nodiscard]] size_t vertex(TriangleVertexId vertex_id) const;

		[[nodiscard]] std::array<size_t, 2> edge_vertices(TriangleEdgeId edge_id) const;
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
	 * Find the facet whose center of gravity is on_which_mesh to the given point,
	 * used as an initial guess for locating the facet on_which_mesh to the point.
	 *
	 * This operation takes O(log n) time in the number of facets.
	 *
	 * @param a 	The point to find the facet on_which_mesh to.
	 * @return 		The index of the facet whose COG is on_which_mesh to the point.
	 */
	size_t guess_closest_face(const Eigen::Vector3d &a) const;

	std::vector<ConvexHullPoint>
	along_cutting_plane(const ConvexHullPoint &start, const ConvexHullPoint &end, const Plane3d &cutting_plane) const;

public:

	/**
	 * Construct a CuttingPlaneConvexHullShell from a convex hull mesh.
	 *
	 * Warning: The mesh given is assumed to already be convex. If it is not,
	 * you may build one using the convexHull function.
	 *
	 * @param mesh 		The mesh to use. (Note: assumed to be convex!)
	 */
	CuttingPlaneConvexHullShell(const shape_msgs::msg::Mesh &mesh, double rotationWeight, double padding);

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
	 * Get the vertices (in Euclidean space) for the given edge of the given face ID.
	 *
	 * @param face_i	The face ID.
	 * @param edge_id 	The edge ID.
	 * @return 			The vertices.
	 */
	std::array<Eigen::Vector3d, 2> facet_edge_vertices(size_t face_i, TriangleEdgeId edge_id) const;

	/**
	 * Compute the outward-pointing normal of a facet. Normalization is applied.
	 *
	 * @param i 	The index of the facet (0 <= i < num_facets()).
	 * @return 		The normal (unit length).
	 */
	Eigen::Vector3d facet_normal(size_t i) const;

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

	Eigen::Vector3d arm_vector(const ConvexHullPoint &p) const override;

	ConvexHullPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	Eigen::Vector3d surface_point(const ConvexHullPoint &p) const override;

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
	std::shared_ptr<ShellPath<ConvexHullPoint>>
	path_from_to(const ConvexHullPoint &from, const ConvexHullPoint &to) const override;

	double path_length(const std::shared_ptr<ShellPath<ConvexHullPoint>> &path) const override;

protected:

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
	double facet_signed_distance(const Eigen::Vector3d& ptr, size_t facet_index) const;

	/**
	 * Compute the "support point" for a convex_hull_walk. That is: the third point to define the cutting plane,
	 * in this case computed by taking the halfway point between a and b and projecting it onto the convex hull.
	 *
	 * @param a		The first point.
	 * @param b		The second point.
	 * @return		The support point.
	 */
	Eigen::Vector3d computeSupportPoint(const ConvexHullPoint &a, const ConvexHullPoint &b) const;

};

std::shared_ptr<WorkspaceShell<ConvexHullPoint>>
cuttingPlaneConvexHullAroundLeaves(const AppleTreePlanningScene &scene_info, double padding, double rotation_weight);


#endif //NEW_PLANNERS_CUTTINGPLANECONVEXHULLSHELL_H
