#include "CuttingPlaneConvexHullShell.h"
#include "../utilities/convex_hull.h"
#include "../utilities/math_utils.h"
#include "../utilities/geogebra.h"
#include "../utilities/experiment_utils.h"
#include "../utilities/enclosing_sphere.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/to_container.hpp>

const double EPSILON = 10e-10;

CuttingPlaneConvexHullShell::CuttingPlaneConvexHullShell(const shape_msgs::msg::Mesh &mesh,
														 double rotationWeight,
														 double padding)
		: rotation_weight(rotationWeight), padding(padding) {

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
CuttingPlaneConvexHullShell::path_from_to(const ConvexHullPoint &a, const ConvexHullPoint &b) const {

	// Find all the facets that are intersected by the cutting plane.
	Eigen::Vector3d support_point = computeSupportPoint(a, b);

	// If a,b,c are collinear, then just return the straight line between a and b.
	if (std::abs((a.position - support_point).norm() + (b.position - support_point).norm() -
				 (a.position - b.position).norm()) < 1e-6) {

		std::vector<ConvexHullPoint> walk = {a, b};
		return std::make_shared<PiecewiseLinearPath<ConvexHullPoint>>(walk);

	} else {

		// We'll perform a simple, A*-like search to find the path.
		Plane3d cutting_plane((a.position - support_point).cross(b.position - support_point).normalized(),
							  support_point);

		return std::make_shared<PiecewiseLinearPath<ConvexHullPoint>>(along_cutting_plane(a, b, cutting_plane));

	}

}

Eigen::Vector3d CuttingPlaneConvexHullShell::computeSupportPoint(const ConvexHullPoint &a, const ConvexHullPoint &b) const {
	return nearest_point_on_shell(0.5 * (a.position + b.position)).position;
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

std::array<Eigen::Vector3d, 2> CuttingPlaneConvexHullShell::facet_edge_vertices(size_t face_i, TriangleEdgeId edge_id) const {
	return {
		vertices[facets[face_i].edge_vertices(edge_id)[0]],
		vertices[facets[face_i].edge_vertices(edge_id)[1]]
	};
}

//Json::Value ConvexHullShellBuilder::parameters() const {
//	Json::Value params;
//
//	params["type"] = "convex_hull";
//	params["rotation_weight"] = rotation_weight;
//	params["padding"] = padding;
//
//	return params;
//}

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

Eigen::Vector3d CuttingPlaneConvexHullShell::arm_vector(const ConvexHullPoint &p) const {
	return -facet_normal(p.face_id);
}

ConvexHullPoint CuttingPlaneConvexHullShell::nearest_point_on_shell(const Eigen::Vector3d &p) const {
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

Eigen::Vector3d CuttingPlaneConvexHullShell::surface_point(const ConvexHullPoint &p) const {
	return p.position;
}

double CuttingPlaneConvexHullShell::path_length(const std::shared_ptr<ShellPath<ConvexHullPoint>> &path) const {

	auto p = std::dynamic_pointer_cast<PiecewiseLinearPath<ConvexHullPoint>>(path);

	assert(p); // I'd rather do this statically, but oh well.

	double length = 0.0;

	for (size_t i = 0; i + 1 < p->points.size(); ++i) {
		length += (p->points[i].position - p->points[i + 1].position).norm();

		double angle_tan = facet_normal(p->points[i].face_id).dot(facet_normal(p->points[i + 1].face_id));
		// We clamp the angle_tan to [-1, 1] to avoid NaNs, since rounding
		// errors can cause it to be slightly outside that range.
		length += std::acos(std::clamp(angle_tan, -1.0, 1.0));
	}

	return length;
}

struct FaceVisit {
	std::shared_ptr<FaceVisit> previous;
	ConvexHullPoint entry_point;
	double distance;
};

bool operator<(const std::shared_ptr<FaceVisit> &a, const std::shared_ptr<FaceVisit> &b) {
	return a->distance > b->distance;
}

std::vector<ConvexHullPoint> CuttingPlaneConvexHullShell::along_cutting_plane(const ConvexHullPoint &a,
																			  const ConvexHullPoint &b,
																			  const Plane3d &cutting_plane) const {

	// We will use Dijkstra's algorithm to find the shortest path between the two points.

	// Create a priority queue and add the starting point.
	std::priority_queue<std::shared_ptr<FaceVisit>> queue;
	queue.push(std::make_shared<FaceVisit>(FaceVisit{.previous = nullptr, .entry_point = a, .distance = 0.0}));

	// Keep track of which faces we have visited, so we don't visit them again.
	std::vector<bool> visited(facets.size(), false);

	// Keep track of the exit face, so we can reconstruct the path.
	// Will be set once we reach the exit face, nullptr until then.
	std::shared_ptr<FaceVisit> exit_face;

	while (!queue.empty()) { // Keep going until we have no more faces to visit.

		// Get the next face to visit.
		auto visit = queue.top();
		queue.pop();

		// If we have already visited this face, skip it. Note that we can't do this on queue insertion,
		// because at that point we'd have to inspect the entire queue to see if the candidate is already
		// in there. It's easier to put the filter here.
		if (visited[visit->entry_point.face_id]) {
			continue;
		}

		// Mark this face as visited.
		visited[visit->entry_point.face_id] = true;

		// If we have reached the exit face, we're done.
		if (visit->entry_point.face_id == b.face_id) {
			exit_face = visit;
			break;
		}

		// Look at every edge of this facet.
		for (TriangleEdgeId edge_id: {TriangleEdgeId::EDGE_AB, TriangleEdgeId::EDGE_BC, TriangleEdgeId::EDGE_CA}) {

			// Get the vertices of this edge.
			const auto &[p, q] = facets[visit->entry_point.face_id].edge_vertices(edge_id);
			auto pt_p = vertices[p];
			auto pt_q = vertices[q];

			// Create a segment from the edge.
			math_utils::Segment3d edge(pt_p, pt_q);

			// Generate a new ConvexHullPoint for the entry point of the next facet.
			ConvexHullPoint exit_point;
			exit_point.face_id = facets[visit->entry_point.face_id].neighbour(edge_id);

			// Find the intersection of the edge with the cutting plane.
			auto intersection = find_intersection(edge, cutting_plane, 1.0e-6);

			if (std::holds_alternative<std::monostate>(intersection)) {
				// No intersection, skip this edge.
				continue;
			} else if (std::holds_alternative<math_utils::Segment3d>(intersection)) {
				// The edge lies in the cutting plane. Use the current point.
				exit_point.position = visit->entry_point.position;
			} else if (std::holds_alternative<Eigen::Vector3d>(intersection)) {
				// The edge intersects the cutting plane. Use the intersection point.
				exit_point.position = std::get<Eigen::Vector3d>(intersection);
			} else {
				throw std::runtime_error("Unexpected intersection type");
			}

			// Compute the distance from the entry point to the exit point.
			double distance = (exit_point.position - visit->entry_point.position).norm();

			FaceVisit new_visit{.previous = visit, .entry_point = exit_point, .distance = visit->distance + distance};

			// Add the exit point to the queue.
			queue.push(std::make_shared<FaceVisit>(new_visit));

		}

	}

	// If we didn't find an exit face, there is no path between the two points.
	// This should never happen, since the surface of a convex hull is connected.
	if (!exit_face) {
		throw std::runtime_error("Could not find path between points");
	}

	// Now we have the exit face, we can walk back to the entry face.
	// We will store the path in a vector.
	std::vector<ConvexHullPoint> walk{b // Start with the exit point.
	};

	while (exit_face->previous) {
		// We're not on the first facet yet.

		// Add the entry point of the current facet.
		walk.push_back(exit_face->entry_point);

		// And the exit point of the previous facet, which is the same point in Euclidean space,
		// but has a different face_id.
		walk.push_back({.face_id = exit_face->previous->entry_point.face_id, .position = exit_face->entry_point
				.position});

		// Move to the previous facet.
		exit_face = exit_face->previous;
	}

	// Add the entry point of the first facet.
	walk.push_back(a);

	// Reverse the path, so it goes from a to b.
	std::reverse(walk.begin(), walk.end());

	return walk;
}

std::shared_ptr<WorkspaceShell<ConvexHullPoint>>
cuttingPlaneConvexHullAroundLeaves(const AppleTreePlanningScene &scene_info, double padding, double rotation_weight) {

	auto leaf_vertices = utilities::extract_leaf_vertices(scene_info);

	return std::make_shared<CuttingPlaneConvexHullShell>(convexHull(leaf_vertices), rotation_weight, padding);

}
