//
// Created by werner on 3-8-22.
//

#ifndef NEW_PLANNERS_MATH_UTILS_H
#define NEW_PLANNERS_MATH_UTILS_H

#include <utility>
#include <Eigen/Geometry>

/**
 * Given two lines, expressed in origin-vector form, find the points of on_which_mesh approach.
 *
 * @param l1 	The first line.
 * @param l2 	The second line.
 * @return 		Parameters t and s, such that l1.pointAt(t) and l2.pointAt(s) are the points of on_which_mesh approach.
 */
std::pair<double, double> closest_point_on_line(
		const Eigen::ParametrizedLine<double, 3>& l1,
		const Eigen::ParametrizedLine<double, 3>& l2);

/**
 * Given a line and a point, find the projection of the point onto the line, expressed as a parameter for the line.
 *
 * @param line 		The line.
 * @param point 	The point.
 * @return 			The parameter t, such that line.pointAt(t) is the projection of the point onto the line.
 */
double projectionParameter(const Eigen::ParametrizedLine<double, 3>& line, const Eigen::Vector3d& point);

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
	EDGE_AB = 0,
	EDGE_BC = 1,
	EDGE_CA = 2
};

enum TriangleVertexId {
	VERTEX_A = 0,
	VERTEX_B = 1,
	VERTEX_C = 2
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
 * Returns whether there exists an intersection between the given sphere and AABB.
 *
 * @param sphere_center 		The center of the sphere.
 * @param sphere_radius 		The radius of the sphere.
 * @param aabb 					The AABB.
 * @return 						True if there exists an intersection, false otherwise.
 */
bool hollow_sphere_intersects_hollow_aabb(const Eigen::Vector3d &sphere_center,
										  double sphere_radius,
										  const Eigen::AlignedBox3d &aabb);


struct OctantIterator {
	size_t i;
	const Eigen::AlignedBox3d bounds;

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

#endif //NEW_PLANNERS_MATH_UTILS_H
