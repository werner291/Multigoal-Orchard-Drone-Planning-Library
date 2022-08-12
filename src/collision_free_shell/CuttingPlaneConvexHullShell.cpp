#include "CuttingPlaneConvexHullShell.h"
#include "../utilities/convex_hull.h"
#include "../utilities/math_utils.h"
#include "../utilities/geogebra.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/numeric/accumulate.hpp>

const double EPSILON = 10e-10;

moveit::core::RobotState
CuttingPlaneConvexHullShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const ConvexHullPoint &a) const {

	const Eigen::Vector3d normal = facet_normal(a.face_id);

	return robotStateFromFacing(drone,
			// Translate the point by the padding times the normal.
								a.position + normal * padding,
			// Facing inward, so opposite the normal.
								-normal);

}

std::vector<moveit::core::RobotState> CuttingPlaneConvexHullShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																				 const ConvexHullPoint &a,
																				 const ConvexHullPoint &b) const {

	// Simply perform a convex hull walk and generate a state for every point.
	std::vector<moveit::core::RobotState> path;

	for (const ConvexHullPoint &pt: convex_hull_walk(a, b)) {
		path.push_back(state_on_shell(drone, pt));
	}

	return path;

}

double CuttingPlaneConvexHullShell::predict_path_length(const ConvexHullPoint &a, const ConvexHullPoint &b) const {

	// I don't like that I'm allocating here... We'll see how bad the bottleneck is.
	// We generate the convex hull walk...
	auto walk = convex_hull_walk(a, b);

	// And simply add up the Euclidean distances and the rotations at edge crossings.
	double length = 0;

	for (size_t i = 1; i < walk.size(); i++) {
		// Add the Euclidean distance between the points.
		length += (walk[i].position - walk[i - 1].position).norm();
		// Add the rotation between the facet normals.
		length += acos(std::clamp(facet_normal(a.face_id).dot(facet_normal(b.face_id)), -1.0, 1.0));
	}

	return length;
}

size_t CuttingPlaneConvexHullShell::guess_closest_face(const Eigen::Vector3d &a) const {
	return facet_index.nearest(NNGNATEntry{SIZE_MAX, a}).face_index;
}

CuttingPlaneConvexHullShell::CuttingPlaneConvexHullShell(const shape_msgs::msg::Mesh &mesh) {

	// Convert the points to Eigen and store.
	this->vertices = mesh.vertices | ranges::views::transform([](const geometry_msgs::msg::Point &p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
	}) | ranges::to_vector;

	// Find a point inside of the mesh.
	Eigen::Vector3d com = ranges::accumulate(vertices, Eigen::Vector3d(0.0, 0.0, 0.0)) / (double) vertices.size();

	this->facets = mesh.triangles | ranges::views::transform([&](const shape_msgs::msg::MeshTriangle &t) {

		// Build a Facet structure, leaving the neighbour_ab, neighbour_bc, and neighbour_ca fields empty for now.
		Facet f = {t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2], 0, 0, 0};

		// Find the normal of the facet (we can't use the facet_normal() function since the facets have not been initialized.
		Eigen::Vector3d normal = (vertices[f.b] - vertices[f.a]).cross(vertices[f.c] - vertices[f.a]).normalized();

		// Ensure winding order puts the normal pointing out from the mesh
		if (normal.dot(vertices[f.a] - com) < 0) {
			std::swap(f.b, f.c);
		}

		return f;

	}) | ranges::to_vector;

	// Match up the faces and populate the neighbour_ab, neighbour_bc, and neighbour_ca fields.
	match_faces();

	// Compute the spatial facet index.
	init_gnat();

}

void CuttingPlaneConvexHullShell::init_gnat() {

	facet_index.setDistanceFunction([](const NNGNATEntry &a, const NNGNATEntry &b) {
		// Distance between facets will be the Euclidean distance between the centroids.
		return (a.at - b.at).norm();
	});

	for (const auto &[i, f]: this->facets | ranges::views::enumerate) {
		// Add the facet to the index, keyed to the centroid.
		facet_index.add(NNGNATEntry{i, (this->vertices[f.a] + this->vertices[f.b] + this->vertices[f.c]) / 3.0});
	}

}

/**
 * A simple structure of two integers (size_t) that can be hashed and checked for equality in an unordered fashion.
 *
 * This will serve as a hash key for the CuttingPlaneConvexHullShell::match_faces function.
 *
 * Ths is: (a,b) == (b,a).
 */
struct UnorderedEdge {
	size_t a, b;

	bool operator==(const UnorderedEdge &other) const {
		// Allow both (a,b) and (b,a) to be equal.
		return a == other.a && b == other.b || a == other.b && b == other.a;
	}
};

template<>
struct std::hash<UnorderedEdge> {
	size_t operator()(const UnorderedEdge &e) const {
		// Using the ^ operator to xor the two integers (we need a commutative hash function).
		return std::hash<size_t>()(e.a) ^ std::hash<size_t>()(e.b);
	}
};

void CuttingPlaneConvexHullShell::match_faces() {

	/// A structure to hold a face index and a pointer to the field in the facet structure that holds the neighbour index.
	struct FaceEdge {
		/// The index of the face.
		size_t face_index;
		/// The pointer to the field in the facet structure that holds the neighbour index.
		size_t *edge_field;
	};

	// For faces that have already been visited, for every edge of that face that has not been matched yet,
	// we keep a key-value pair. The Key is the edge (unordered pair of the vertex indices), the Value is
	// a structure that points back to the facet that was previously known to have this edge.
	std::unordered_map<UnorderedEdge, FaceEdge> edge_to_face;

	for (size_t i = 0; i < facets.size(); i++) {

		// Look up the facet.
		Facet &f = facets[i];

		// Build an array of the edges of the face, with a reference to the neighbour field.
		std::array<std::tuple<size_t, size_t, size_t *>, 3> edges = {std::make_tuple(f.a, f.b, &f.neighbour_ab),
																	 std::make_tuple(f.b, f.c, &f.neighbour_bc),
																	 std::make_tuple(f.c, f.a, &f.neighbour_ca)};

		// For each edge of the face, if it has not been matched yet, match it.
		for (auto [a, b, neighbour]: edges) {

			UnorderedEdge edge{a, b}; // The edge to match.

			if (edge_to_face.find(edge) == edge_to_face.end()) {
				// This edge has not been seen yet, store it for later matching.
				edge_to_face[edge] = {i, neighbour};
			} else {
				// We found a matching edge!
				// Set the neighbour field of the face to the index of the other face.
				*neighbour = edge_to_face[edge].face_index;
				// ...and fill in the back-pointer.
				*edge_to_face[edge].edge_field = i;

				// Every edge can only be matched once (hopefully), so we can remove the edge from the map.
				edge_to_face.erase(edge);
			}
		}

	}
}

ConvexHullPoint CuttingPlaneConvexHullShell::project(const Eigen::Vector3d &p) const {

	// TODO: This is a brute-force O(n) algorithm. We should use some kind of spatial data structure to speed this up.

	size_t face_index;
	double expected_signed_dist = INFINITY;
	Eigen::Vector3d closest;

	for (size_t face_i = 0; face_i < num_facets(); ++face_i) {
		const auto &[va, vb, vc] = facet_vertices(face_i);

		Eigen::Vector3d candidate_closest = closest_point_on_triangle(p, va, vb, vc);

		double distance = (p - candidate_closest).squaredNorm();

		if (distance < expected_signed_dist) {
			expected_signed_dist = distance;
			closest = candidate_closest;
			face_index = face_i;
		}
	}

	return ConvexHullPoint{face_index, closest};

}

ConvexHullPoint CuttingPlaneConvexHullShell::gaussian_sample_near_point(const ConvexHullPoint &near) const {

	const Facet &f = facets[near.face_id];

	ompl::RNG rng;

	Eigen::Vector3d a_rand =
			near.position + Eigen::Vector3d(rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1));

	return project(a_rand);

}

ConvexHullPoint CuttingPlaneConvexHullShell::project(const moveit::core::RobotState &st) const {
	return project(st.getGlobalLinkTransform("base_link").translation());
}

ConvexHullPoint CuttingPlaneConvexHullShell::project(const Apple &st) const {
	return project(st.center);
}

std::vector<ConvexHullPoint>
CuttingPlaneConvexHullShell::convex_hull_walk(const ConvexHullPoint &a, const ConvexHullPoint &b) const {


	if (a.face_id == b.face_id) {
		// This is trivial: the walk is just the two points.
		return {a, b};
	} else {

		// A quick-and-dirty greedy approximation of the geodesic distance between the two points on the convex hull.
		// We start from point A, then "walk" along the surface of the convex hull, and then end up at point B.

		// We first compute a cutting plane. The "walk" will be part of the intersection of the cutting plane with the
		// convex hull, between points A and B.
		Eigen::Vector3d support_point = computeSupportPoint(a, b);
		const auto cutting_plane = plane_from_points(a.position, b.position, support_point);

		// The current walk, which we will gradually build up as the algorithm iterates.
		std::vector<ConvexHullPoint> walk{a};

		// Keep going until we reach the end point.
		while (walk.back().face_id != b.face_id && (walk.back().position - b.position).squaredNorm() > 1e-10) {

			// Compute the next step.
			ConvexHullPoint next_point = walk_step(b, support_point, cutting_plane, walk);

			// To ensure smooth interpolation, we split the next step between one that moves to the exit point
			// of the current facet, then transition sto the new facet (likely with a different normal).
			walk.push_back(ConvexHullPoint{walk.back().face_id, next_point.position});
			walk.push_back(next_point);

		}

		// Push the end point onto the walk.
		walk.push_back(b);

		return walk;
	}
}

std::optional<ConvexHullPoint> CuttingPlaneConvexHullShell::step_through_edge_in_cutting_plane(const ConvexHullPoint &b,
																							   const Eigen::Vector3d &support_point,
																							   const Plane3d &cutting_plane,
																							   const std::vector<ConvexHullPoint> &walk) const {

	// If of the face the walk is currently on.
	size_t current_face = walk.back().face_id;

	for (auto edge: {EDGE_AB, EDGE_BC, EDGE_CA}) {

		// Look up the Carthesian coordinates of the two vertices of the edge.
		const auto [vp, vq] = facet_edge_vertices(current_face, edge);

		// Check if the edge fully lies in the plane (by both vertices lying on the plane).
		if (cutting_plane.absDistance(vp) < EPSILON && cutting_plane.absDistance(vq) < EPSILON) {

			// Project the goal point onto the line that extends the edge.
			Eigen::ParametrizedLine<double, 3> edge_line(vp, vq - vp);

			// Restrict so we don't overshoot the edges of the triangle.
			double t = std::clamp(projectionParameter(edge_line, b.position), 0.0, 1.0);

			// Step to that point, and across the edge.
			// (FIXME: this might get into an infinite loop if we have colinear edges in the convex hull)
			return {ConvexHullPoint{facets[current_face].neighbour(edge), edge_line.pointAt(t)}};
		}
	}

	return std::nullopt;

}

ConvexHullPoint CuttingPlaneConvexHullShell::walk_step(const ConvexHullPoint &b,
													   const Eigen::Vector3d &support_point,
													   const Plane3d &cutting_plane,
													   const std::vector<ConvexHullPoint> &walk) const {

	if (auto edge_step = step_through_edge_in_cutting_plane(b, support_point, cutting_plane, walk)) {
		return *edge_step;
	} else if (walk.size() == 1) {
		// This is the first step on the walk. We don't have to worry about stepping back as much
		// as we should worry about departing in the right direction.
		return firstStep(support_point, cutting_plane, walk.back());
	} else {
		return nextStep(walk.back(), cutting_plane, walk[walk.size() - 3].face_id);
	}
}

std::optional<ConvexHullPoint>
CuttingPlaneConvexHullShell::strictEdgeTraversal(size_t face_id, TriangleEdgeId edge, const Plane3d &cutting_plane) const {

	// ... look up the Carthesian coordinates of the two vertices of the edge,
	const auto [vp, vq] = facet_edge_vertices(face_id, edge);

	// ... compute the intersection of the extension of the edge with the cutting plane
	Eigen::ParametrizedLine<double, 3> edge_line(vp, vq - vp);
	double t = edge_line.intersectionParameter(cutting_plane);
	Eigen::Vector3d pt = edge_line.pointAt(t);

	if (0.0 + EPSILON < t && t + EPSILON < 1.0) {
		return {{facet(face_id).neighbour(edge), pt}};
	} else {
		return {};
	}
}

std::optional<ConvexHullPoint> CuttingPlaneConvexHullShell::firstStepThroughEdges(const Eigen::Vector3d &towards_point,
																				  const Plane3d &cutting_plane,
																				  const ConvexHullPoint &start_point) const {

	// Id of the face the walk is currently on.
	size_t current_face = start_point.face_id;

	ConvexHullPoint next_point;
	double d_min = INFINITY;

	// For every edge...
	for (auto edge: {EDGE_AB, EDGE_BC, EDGE_CA}) {
		if (auto pt = strictEdgeTraversal(current_face, edge, cutting_plane)) {
			// ... compute the distance to the edge.
			double d = (pt->position - towards_point).norm();
			if (d < d_min) {
				d_min = d;
				next_point = *pt;
			}
		}
	}

	// If we found a point, return it.
	if (d_min < INFINITY) {
		return next_point;
	} else {
		// There is no clean intersection.
		return std::nullopt;
	}

}


ConvexHullPoint CuttingPlaneConvexHullShell::firstStep(const Eigen::Vector3d &towards_point,
													   const Plane3d &cutting_plane,
													   const ConvexHullPoint &start_point) const {

	size_t current_face = start_point.face_id;

	if (auto through_edges = firstStepThroughEdges(towards_point, cutting_plane, start_point)) {
		return *through_edges;
	} else {
		const auto &[va, vb, vc] = facet_vertices(current_face);

		// We must be at a corner. Pick the edge whose middle point is closest.
		for (TriangleVertexId v: {TriangleVertexId::VERTEX_A, TriangleVertexId::VERTEX_B, TriangleVertexId::VERTEX_C}) {

			auto f = facet(current_face);

			if (cutting_plane.absDistance(vertex(f.vertex(v))) < EPSILON) {
				return stepAroundVertexTowards(towards_point, current_face, v);
			}
		}

		throw std::runtime_error("No corner on the cutting plane. Is there an intersection at all?");
	}


}

ConvexHullPoint CuttingPlaneConvexHullShell::stepAroundVertexTowards(const Eigen::Vector3d &towards_point,
																	 size_t current_face,
																	 TriangleVertexId &v) const {
	// Look up the facet.
	const auto &f = facet(current_face);

	// Find the edges adjacent to the vertex
	auto adjacent = edges_adjacent_to_vertex(v);

	// Look up their vertices
	auto [p1, q1] = facet_edge_vertices(current_face, adjacent[0]);
	auto [p2, q2] = facet_edge_vertices(current_face, adjacent[1]);

	// Compute the distance of their middle point to the direction indicator point
	double d1 = (towards_point - ((p1 + q1) / 2.0)).squaredNorm();
	double d2 = (towards_point - ((p2 + q2) / 2.0)).squaredNorm();

	// Return a step to the closest one, using the vertex itself as the (carthesian) point being stepped to.
	if (d1 < d2) {
		return {f.neighbour(adjacent[0]), vertex(f.vertex(v))};
	} else {
		return {f.neighbour(adjacent[1]), vertex(f.vertex(v))};
	}
}

Eigen::Vector3d CuttingPlaneConvexHullShell::computeSupportPoint(const ConvexHullPoint &a, const ConvexHullPoint &b) const {
	// Grab the middle point and project it onto the convex hull
	ConvexHullPoint middle_proj = project(0.5 * (a.position + b.position));
	// Compute the normal at that point
	Eigen::Vector3d middle_normal = facet_normal(middle_proj.face_id);
	// Send it off along the normal a bit to ensure we avoid colinear points.
	Eigen::Vector3d middle_proj_euc = middle_proj.position + middle_normal;
	return middle_proj_euc;
}

size_t CuttingPlaneConvexHullShell::num_facets() const {
	return facets.size();
}

const CuttingPlaneConvexHullShell::Facet &CuttingPlaneConvexHullShell::facet(size_t i) const {
	return facets[i];
}

const Eigen::Vector3d &CuttingPlaneConvexHullShell::vertex(size_t i) const {
	return vertices[i];
}

std::array<Eigen::Vector3d, 3> CuttingPlaneConvexHullShell::facet_vertices(size_t facet_i) const {
	const Facet &f = facet(facet_i);

	return {vertex(f.a), vertex(f.b), vertex(f.c)};
}

double CuttingPlaneConvexHullShell::facet_signed_distance(const Eigen::Vector3d &ptr, size_t facet_index) const {
	const auto &[a, b, c] = facet_vertices(facet_index);

	Eigen::Vector3d ab = b - a;
	Eigen::Vector3d ac = c - a;

	Eigen::Vector3d n = ab.cross(ac);

	return n.dot(ptr - a);
}

double CuttingPlaneConvexHullShell::signed_distance(const Eigen::Vector3d &pt) const {
	double d = -INFINITY;

	for (size_t i = 0; i < facets.size(); i++) {
		d = std::max(d, facet_signed_distance(pt, i));
	}

	return d;
}

Eigen::Vector3d CuttingPlaneConvexHullShell::facet_normal(size_t i) const {
	const auto &[a, b, c] = facet_vertices(i);

	return (b - a).cross(c - a).normalized();
}

ConvexHullPoint CuttingPlaneConvexHullShell::nextStep(const ConvexHullPoint &current_point,
													  const Plane3d &cutting_plane,
													  size_t last_face_id) const {

	// Look up the facet data.
	const Facet &f = facet(current_point.face_id);

	// Check if there is a clean intersection with an edge that doesn't lead to the facet we just came from.
	for (auto edge: {EDGE_AB, EDGE_BC, EDGE_CA}) {
		if (f.neighbour(edge) != last_face_id) {
			if (auto intersection = strictEdgeTraversal(current_point.face_id, edge, cutting_plane)) {
				// We found one! This is the easy case, we just return it.
				return *intersection;
			}
		}
	}

	// There was no clean interior intersection with one of the edges... We must be intersecting at a vertex!
	for (auto vertex: {VERTEX_A, VERTEX_B, VERTEX_C}) {
		if (cutting_plane.absDistance(vertices[f.vertex(vertex)]) < 1e-6) {
			// We found the vertex we're intersecting with (which coincides with it lying on the plane).

			for (auto edge: edges_adjacent_to_vertex(vertex)) {
				// We step through the edge adjacent to the vertex that leads to a different facet than the one we came from.
				if (f.neighbour(edge) != last_face_id) {
					return {f.neighbour(edge), vertices[f.vertex(vertex)]};
				}
			}
		}
	}

	throw std::runtime_error("Could not find an exit point. There may not be an intersection?");
}

std::array<Eigen::Vector3d, 2> CuttingPlaneConvexHullShell::facet_edge_vertices(size_t face_i, TriangleEdgeId edge_id) const {
	return {
		vertices[facets[face_i].edge_vertices(edge_id)[0]],
		vertices[facets[face_i].edge_vertices(edge_id)[1]]
	};
}

std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const AppleTreePlanningScene &scene_info) {
	std::vector<geometry_msgs::msg::Point> mesh_points;
	for (const auto &col: scene_info.scene_msg.world.collision_objects) {
		if (col.id == "leaves") {
			for (const auto &mesh: col.meshes) {
				for (auto v: mesh.vertices) {
					mesh_points.push_back(v);
				}
			}
		}
	}
	return mesh_points;
}

std::shared_ptr<OMPLSphereShellWrapper<ConvexHullPoint>>
ConvexHullShellBuilder::buildShell(const AppleTreePlanningScene &scene_info,
								   const ompl::base::SpaceInformationPtr &si) const {

	auto leaf_vertices = extract_leaf_vertices(scene_info);

	return std::make_shared<OMPLSphereShellWrapper<ConvexHullPoint>>(std::make_shared<CuttingPlaneConvexHullShell>(convexHull(
			leaf_vertices)), si);

}


Json::Value ConvexHullShellBuilder::parameters() const {
	Json::Value params;

	params["type"] = "convex_hull";

	return params;
}

bool CuttingPlaneConvexHullShell::NNGNATEntry::operator==(const CuttingPlaneConvexHullShell::NNGNATEntry &rhs) const {
	return face_index == rhs.face_index && at == rhs.at;
}

bool CuttingPlaneConvexHullShell::NNGNATEntry::operator!=(const CuttingPlaneConvexHullShell::NNGNATEntry &rhs) const {
	return !(rhs == *this);
}

std::array<size_t, 3> CuttingPlaneConvexHullShell::Facet::neighbours() const {
	return {neighbour_ab, neighbour_bc, neighbour_ca};
}

size_t CuttingPlaneConvexHullShell::Facet::neighbour(TriangleEdgeId edge_id) const {
	switch (edge_id) {
		case EDGE_AB:
			return neighbour_ab;
		case EDGE_BC:
			return neighbour_bc;
		case EDGE_CA:
			return neighbour_ca;
		default:
			throw std::runtime_error("Invalid edge id");
	}
}

size_t CuttingPlaneConvexHullShell::Facet::vertex(TriangleVertexId vertex_id) const {
	switch (vertex_id) {
		case TriangleVertexId::VERTEX_A:
			return a;
		case TriangleVertexId::VERTEX_B:
			return b;
		case TriangleVertexId::VERTEX_C:
			return c;
	}
	throw std::runtime_error("Invalid vertex id");
}

std::array<size_t, 2> CuttingPlaneConvexHullShell::Facet::edge_vertices(TriangleEdgeId edge_id) const {
	switch (edge_id) {
		case TriangleEdgeId::EDGE_AB:
			return {a, b};
		case TriangleEdgeId::EDGE_BC:
			return {b, c};
		case TriangleEdgeId::EDGE_CA:
			return {c, a};
	}
	throw std::runtime_error("Invalid edge id");
}
