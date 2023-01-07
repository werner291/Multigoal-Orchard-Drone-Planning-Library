//
// Created by werner on 3-8-22.
//

#ifndef NEW_PLANNERS_MATH_UTILS_H
#define NEW_PLANNERS_MATH_UTILS_H

#include <random>
#include <iostream>
#include <utility>
#include <Eigen/Geometry>
#include <optional>
#include <variant>
#include "EigenExt.h"

/**
 * Given two lines, expressed in origin-vector form, find the points of on_which_mesh approach.
 *
 * @param l1 	The first line.
 * @param l2 	The second line.
 * @return 		Parameters t and s, such that l1.pointAt(t) and l2.pointAt(s) are the points of on_which_mesh approach.
 */
std::pair<double, double>
closest_point_on_line(const Eigen::ParametrizedLine<double, 3> &l1, const Eigen::ParametrizedLine<double, 3> &l2);

/**
 * Given a line and a point, find the projection of the point onto the line, expressed as a parameter for the line.
 *
 * @param line 		The line.
 * @param point 	The point.
 * @return 			The parameter t, such that line.pointAt(t) is the projection of the point onto the line.
 */
double projectionParameter(const Eigen::ParametrizedLine<double, 3> &line, const Eigen::Vector3d &point);

/**
 *
 * Compute the barycentric coordinates of a point in the plane defined by the three vertices of a triangle.
 *
 * Based on StackExchange answer here: https://math.stackexchange.com/a/2579920
 *
 * The function returns a vector (alpha,beta,gamma) such that qp' == alpha*va + beta*vb + gamma*vc where
 * qp' is the projection of the query point into the plane defined by the three vertices of the triangle.
 *
 * @param qp	The point to compute the barycentric coordinates of.
 * @param va	The first vertex of the triangle.
 * @param vb	The second vertex of the triangle.
 * @param vc	The third vertex of the triangle.
 * @return		The barycentric coordinates of the point (alpha, beta, gamma).
 */
Eigen::Vector3d project_barycentric(const Eigen::Vector3d &qp,
									const Eigen::Vector3d &va,
									const Eigen::Vector3d &vb,
									const Eigen::Vector3d &vc);


Eigen::Vector3d closest_point_on_triangle(const Eigen::Vector3d &p,
										  const Eigen::Vector3d &va,
										  const Eigen::Vector3d &vb,
										  const Eigen::Vector3d &vc);

struct OpenTriangle {
	Eigen::Vector3d apex;
	Eigen::Vector3d dir1;
	Eigen::Vector3d dir2;

	[[nodiscard]] EigenExt::UVector3d normal() const {
		return EigenExt::UVector3d(dir1.cross(dir2).normalized());
	}
};

/*
 * Compute the closest point on an "open triangle", the triangle defined by a pair of rays with a common origin.
 */
Eigen::Vector3d closest_point_on_open_triangle(const Eigen::Vector3d &p, const OpenTriangle &triangle);

/**
 * If the point p is within a given margin distance from one of the vertices of the triangle,
 * return a point slightly towards the other two vertices.
 *
 * @param p 		The point to check.
 * @param va 		The first vertex of the triangle.
 * @param vb 		The second vertex of the triangle.
 * @param vc 		The third vertex of the triangle.
 * @param margin 	The margin distance.
 * @return 			A point within the triangle at least margin distance from the vertices near p.
 */
Eigen::Vector3d cheat_away_from_vertices(const Eigen::Vector3d &p,
										 const Eigen::Vector3d &va,
										 const Eigen::Vector3d &vb,
										 const Eigen::Vector3d &vc,
										 const double margin = 1.0e-6);

using Plane3d = Eigen::Hyperplane<double, 3>;

/**
 * Compute a plane passing through three points.
 *
 * @param p1 	The first point.
 * @param p2 	The second point.
 * @param p3 	The third point.
 * @return 		The plane passing through the three points.
 */
Plane3d plane_from_points(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3);

enum TriangleEdgeId {
	EDGE_AB = 0, EDGE_BC = 1, EDGE_CA = 2
};

enum TriangleVertexId {
	VERTEX_A = 0, VERTEX_B = 1, VERTEX_C = 2
};

std::array<TriangleVertexId, 2> vertices_in_edge(TriangleEdgeId edge);

std::array<TriangleEdgeId, 2> edges_adjacent_to_vertex(TriangleVertexId vertex);

/**
 * Pick a point universally at random on the surface of a triangle.
 *
 * @param p1 		The first vertex of the triangle.
 * @param p2 		The second vertex of the triangle.
 * @param p3 		The third vertex of the triangle.
 * @return 			A point uniformly at random on the surface of the triangle.
 */
Eigen::Vector3d
uniform_point_on_triangle(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3);

/**
 * Returns whether there exists an find_intersection between the given sphere and AABB.
 *
 * @param sphere_center 		The center of the sphere.
 * @param sphere_radius 		The radius of the sphere.
 * @param aabb 					The AABB.
 * @return 						True if there exists an find_intersection, false otherwise.
 */
bool hollow_sphere_intersects_hollow_aabb(const Eigen::Vector3d &sphere_center,
										  double sphere_radius,
										  const Eigen::AlignedBox3d &aabb);

/**
 *
 * Iterator over the octants of a Eigen::AlignedBox3d.
 *
 * Specifically, in the order:
 *
 * 1. +x;+y;+z
 * 2. -x;+y;+z
 * 3. +x;-y;+z
 * 4. -x;+y;+z
 * 5. +x;+y;-z
 * 6. -x;+y;-z
 * 7. +x;-y;-z
 * 8. -x;+y;-z
 */
struct OctantIterator {
	size_t i;
	const Eigen::AlignedBox3d total_box;

private:
	OctantIterator(size_t i, const Eigen::AlignedBox3d &bounds);

public:

	explicit OctantIterator(const Eigen::AlignedBox3d &bounds);

	static OctantIterator end();

	OctantIterator &operator++();

	OctantIterator operator++(int);

	bool operator==(const OctantIterator &other) const;

	bool operator!=(const OctantIterator &other) const;

	Eigen::AlignedBox3d operator*() const;

};


namespace math_utils {

	/**
	 * @brief Finds the octant containing the given point in the given bounding box.
	 * @param box The bounding box to search within.
	 * @param point The point to search for.
	 * @return The OctantIterator of the octant containing the point.
	 */
	OctantIterator find_octant_containing_point(const Eigen::AlignedBox3d &box, const Eigen::Vector3d &point);

	/**
	 * Returns the parameter value of the given parametrized line at the plane defined by the given
	 * dimension and value. If the line is parallel to the plane, returns NaN.
	 *
	 * @param p The parametrized line.
	 * @param d The dimension of the plane.
	 * @param value The value at the given dimension of the plane.
	 * @return The parameter value at the plane, or NaN if the line is parallel to the plane.
	 */
	double param_at_plane(const EigenExt::ParametrizedLine3d &p, size_t d, double value);

	struct Segment3d {
		Eigen::Vector3d start;
		Eigen::Vector3d end;

		Segment3d(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

		/**
		 * Compute the point on the segment closest to the given point.
		 *
		 * @param p 	The point to compute the closest point to.
		 * @return 		The point on the segment closest to p.
		 */
		[[nodiscard]] Eigen::Vector3d closest_point(const Eigen::Vector3d &p) const;

		[[nodiscard]] Eigen::Vector3d displacement() const;

		[[nodiscard]] Eigen::ParametrizedLine<double, 3> extended_line() const;

	};

	/**
	 * @brief Check whether the given AABB fully contains the given line segment.
	 */
	bool box_contains_segment(const Eigen::AlignedBox3d &box, const Segment3d &segment);

	/**

    @brief Check if a given point is closer to the center of the given bounding box than the edge length.
    @param point The point to check.
    @param box The bounding box to check within.
    @return true if the point is closer to the center of the box than the edge length, false otherwise.
    */
	bool point_closer_to_center_than_edge(const Eigen::Vector3d &point, const Eigen::AlignedBox3d &box);

	/**
 	 * @brief Computes the find_intersection parameters of the given line segment with the given AABB.
     * If the line segment intersects the AABB, returns the parameters at which the line intersects
     * the minimum and maximum corner of the AABB. If the line segment does not intersect the AABB,
     * returns an empty optional.
     * @param box The AABB to intersect with the line segment.
     * @param segment The line segment to intersect with the AABB.
     * @return The find_intersection parameters of the line segment with the AABB, or an empty optional if no find_intersection.
     */
	std::optional<std::array<double, 2>>
	line_aabb_intersection_params(const Eigen::AlignedBox3d &box, const EigenExt::ParametrizedLine3d &segment);

	/**
	 * This function checks whether a hollow sphere intersects with a hollow axis-aligned bounding box (AABB).
	 *
	 * @param sphere_center The center point of the sphere.
	 * @param sphere_radius The radius of the sphere.
	 * @param aabb The AABB to check for find_intersection with the sphere.
	 *
	 * @return True if the sphere intersects with the AABB, false otherwise.
	 */
	bool hollow_sphere_intersects_hollow_aabb(const Eigen::Vector3d &sphere_center,
											  const double sphere_radius,
											  const Eigen::AlignedBox3d &aabb);



	/**
	 * Finds the point in the given list of points that is closest to the given point.
	 *
	 * @param points The list of points to search for the closest point.
	 * @param p The point to find the closest point to.
	 *
	 * @return The point in the list that is closest to p.
	 */
	Eigen::Vector3d closest_point_in_list(std::initializer_list<Eigen::Vector3d> points, const Eigen::Vector3d &p);

	struct Ray3d {
		Eigen::Vector3d origin;
		Eigen::Vector3d direction;

		Ray3d(const Eigen::Vector3d &origin, const Eigen::Vector3d &direction);

		/**
		 * Compute the point on the ray closest to the given point.
		 *
		 * @param p 	The point to compute the closest point to.
		 * @return 		The point on the ray closest to p.
		 */
		[[nodiscard]] Eigen::Vector3d closest_point(const Eigen::Vector3d &p) const;

		/**
		 * Extend to a ParametrizedLine3d.
		 *
		 * @return The extended line.
		 */
		[[nodiscard]] EigenExt::ParametrizedLine3d extended_line() const;
	};

	/**
	  *	@brief Determines whether the given line segment intersects the given AABB.
	  *	@param box The AABB to test for find_intersection with the line segment.
	  *	@param segment The line segment to test for find_intersection with the AABB.
	  *	@return True if the line segment intersects the AABB, false otherwise.
	  */
	bool segment_intersects_aabb(const Eigen::AlignedBox3d &box, const Segment3d &segment);

	/**
	 * @brief Return the find_intersection of the given line with the given plane.
	 * @param segment The line to intersect with the plane.
	 * @param plane The plane to intersect with the line segment.
	 * @return Variant of either nothing, the find_intersection point (as a parameter), or the line segment if the line segment is coplanar with the plane.
	 */
	std::variant<std::monostate, double, Eigen::ParametrizedLine<double, 3>> find_intersection(
			const EigenExt::ParametrizedLine3d &segment, const Eigen::Hyperplane<double, 3> &plane);


	/**
	 * @brief Return the find_intersection of the given line segment with the given plane.
	 * @param segment The line segment to intersect with the plane.
	 * @param plane The plane to intersect with the line segment.
	 * @return Variant of either nothing, the find_intersection point, or the line segment if the line segment is coplanar with the plane.
	 */
	std::variant<std::monostate, Eigen::Vector3d, Segment3d>
	find_intersection(const Segment3d &segment, const Eigen::Hyperplane<double, 3> &plane);

	/**
	 * Return whether given ray intersects given AABB.
	 * @param box 			The AABB to test for find_intersection with the ray.
	 * @param ray3D 		The ray to test for find_intersection with the AABB.
	 *
	 * @return 				True if the ray intersects the AABB, false otherwise.
	 */
	bool intersects(const Eigen::AlignedBox3d &box, const Ray3d &ray3D);

	struct ViewPyramidFaces {
		OpenTriangle left;
		OpenTriangle right;
		OpenTriangle top;
		OpenTriangle bottom;
	};

	/**
	 * @brief Computes the view pyramid planes for the given camera.
	 * @param tf The camera transform.
	 * @param fovX The horizontal field of view of the camera.
	 * @param fovY The vertical field of view of the camera.
	 */
	ViewPyramidFaces compute_view_pyramid_planes(const Eigen::Isometry3d &tf, double fovX, double fovY);

	/**
	 * @brief Test whether the given AABB intersects with a plane.
	 * @param box 		The AABB to test for find_intersection with the plane.
	 * @param plane 	The plane to test for find_intersection with the AABB.
	 * @return 			True if the plane intersects the AABB, false otherwise.
	 */
	bool intersects(const Eigen::AlignedBox3d &box, const EigenExt::Plane3d &plane);

	struct Triangle3d {
		Eigen::Vector3d a;
		Eigen::Vector3d b;
		Eigen::Vector3d c;
	};

	struct Quad3d {
		Eigen::Vector3d a;
		Eigen::Vector3d b;
		Eigen::Vector3d c;
		Eigen::Vector3d d;
	};

	std::array<Segment3d, 12> aabb_edges(const Eigen::AlignedBox3d &box);

	/**
		 * Return the find_intersection of a plane and an AABB, as the section of the plane inside of the AABB.
		 *
		 * @param box 			The AABB to intersect with the plane.
		 * @param plane 		The plane to intersect with the AABB.
		 * @return 				The find_intersection of the plane and the AABB, as a variant of either nothing, a triangle or a quad.
		 */
	std::variant<std::monostate, Triangle3d, Quad3d> find_intersection(const Eigen::AlignedBox3d &box, const EigenExt::Plane3d &plane);
}

/**
 * Among a number of points, find the one closest to a given point.
 * @param points 		The points to search.
 * @param p 			The point to search for.
 * @return 				The index of the point closest to p.
 */
Eigen::Vector3d closest_point_in_list(std::initializer_list<Eigen::Vector3d> points, const Eigen::Vector3d &p);

#if JSON_FUNCTIONS

#include <json/json.h>

/**
 * Convert an Eigen::Vector3d to a JSON array
 *
 * @param v  Eigen::Vector3d
 * @return JSON array
 */
Json::Value toJSON(const Eigen::Vector3d &v);

/**
 * Convert a JSON array to an Eigen::Vector3d
 *
 * @param json  JSON array
 * @return Eigen::Vector3d
 */
Eigen::Vector3d fromJsonVector3d(const Json::Value &json);

/**
 * Convert an Eigen::Quaterniond to a JSON array
 *
 * @param q  Eigen::Quaterniond
 * @return JSON array
 */
Json::Value toJSON(const Eigen::Quaterniond &q);

/**
 * Convert a JSON array to an Eigen::Quaterniond
 *
 * @param json  JSON array
 * @return Eigen::Quaterniond
 */
Eigen::Quaterniond fromJsonQuaternion3d(const Json::Value &json);

/**
 * Convert an Eigen::Isometry3d to a JSON object
 *
 * @param isom  Eigen::Isometry3d
 * @return JSON object
 */
Json::Value toJSON(const Eigen::Isometry3d &isom);

/**
 * Convert a JSON object to an Eigen::Isometry3d
 *
 * @param json  JSON object
 * @return Eigen::Isometry3d
 */
Eigen::Isometry3d fromJsonIsometry3d(const Json::Value &json);



#endif

#endif //NEW_PLANNERS_MATH_UTILS_H
