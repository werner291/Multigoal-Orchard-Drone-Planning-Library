#include "ConvexHullShell.h"
#include "../utilities/convex_hull.h"
#include "../utilities/math_utils.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/numeric/accumulate.hpp>

moveit::core::RobotState
ConvexHullShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const ConvexHullPoint &a) const {

	const Eigen::Vector3d normal = facet_normal(a.face_id);

	return robotStateFromFacing(drone,
			// Translate the point by the padding times the normal.
								a.position + normal * padding,
			// Facing inward, so opposite the normal.
								-normal);

}

std::vector<moveit::core::RobotState> ConvexHullShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																	 const ConvexHullPoint &a,
																	 const ConvexHullPoint &b) const {

	// Simply perform a convex hull walk and generate a state for every point.
	std::vector<moveit::core::RobotState> path;

	for (const ConvexHullPoint &pt: convex_hull_walk(a, b)) {
		path.push_back(state_on_shell(drone, pt));
	}

	return path;

}

double ConvexHullShell::predict_path_length(const ConvexHullPoint &a, const ConvexHullPoint &b) const {

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

size_t ConvexHullShell::guess_closest_face(const Eigen::Vector3d &a) const {
	return facet_index.nearest(NNGNATEntry{SIZE_MAX, a}).face_index;
}

ConvexHullShell::ConvexHullShell(const shape_msgs::msg::Mesh &mesh) {

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

void ConvexHullShell::init_gnat() {

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
 * This will serve as a hash key for the ConvexHullShell::match_faces function.
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

void ConvexHullShell::match_faces() {

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
		std::array<std::tuple<size_t, size_t, size_t*>, 3> edges =
				{std::make_tuple(f.a, f.b, &f.neighbour_ab)
				,std::make_tuple(f.b, f.c, &f.neighbour_bc)
				,std::make_tuple(f.c, f.a, &f.neighbour_ca)};

		// For each edge of the face, if it has not been matched yet, match it.
		for (auto[a,b,neighbour] : edges) {
			
			UnorderedEdge edge {a,b}; // The edge to match.

			if (edge_to_face.find(edge) == edge_to_face.end()) {
				// This edge has not been seen yet, store it for later matching.
				edge_to_face[edge] = {i, neighbour};
			} else {
				// We found a matching edge!
				// Set the neighbour field of the face to the index of the other face.
				f.neighbour_ab = edge_to_face[edge].face_index;
				// ...and fill in the back-pointer.
				*edge_to_face[edge].edge_field = i;

				// Every edge can only be matched once (hopefully), so we can remove the edge from the map.
				edge_to_face.erase(edge);
			}
		}

	}
}


ConvexHullPoint ConvexHullShell::project(const Eigen::Vector3d &p) const {

	std::cout << "Query = (" << p.x() << ", " << p.y() << ", " << p.z() << ")" << std::endl;

	size_t face_index = guess_closest_face(p);

	// Find the closest point on the face to the point.

	const auto& [va, vb, vc] = facet_vertices(face_index);

	Eigen::Vector3d closest_point = closest_point_on_triangle(p, va, vb, vc);

	bool improved = false;

	double closest_distance = (closest_point - p).squaredNorm();

	do {

		std::cout << "(" << closest_point.x() << ", " << closest_point.y() << ", " << closest_point.z() << ")" << std::endl;
		std::cout << "Polygon({(" << va.x() << "," << va.y() << "," << va.z() << "),(" << vb.x() << "," << vb.y() << "," << vb.z() << "),(" << vc.x() << "," << vc.y() << "," << vc.z() << ")})" << std::endl;

		improved = false;

		for (size_t neighbour: {facets[face_index].neighbour_ab, facets[face_index].neighbour_bc,
								facets[face_index].neighbour_ca}) {

			const auto &[nva, nvb, nvc] = facet_vertices(neighbour);

			Eigen::Vector3d closest_point_on_neighbour = closest_point_on_triangle(p, nva, nvb, nvc);

			double d = (closest_point_on_neighbour - p).squaredNorm();

			if (d < closest_distance) {
				closest_point = closest_point_on_neighbour;
				closest_distance = d;
				face_index = neighbour;
				improved = true;
			}

		}
	} while (improved);

	return ConvexHullPoint{face_index, closest_point};
//
//	// Take an initial guess by finding the facet whose centroid is closest to the point.
//	size_t face_index = guess_closest_face(a);
//
//
//	// Reference to the facet visited on the previous iteration, using SIZE_MAX as a "null" value.
//	size_t last_face = SIZE_MAX;
//
//	// We terminate by returning our find.
//	while (true) {
//
//		// Reference to the facet we are currently visiting.
//		const Facet &f = facets[face_index];
//
//		const auto& [va, vb, vc] = facet_vertices(face_index);
//
//		// Project the point onto the plane defined by the facet, expressed as barycentric coordinates.
//		Eigen::Vector3d a_barycentric = project_barycentric(a, va, vb, vc);
//
//		// A point lies within a facet if all three barycentric coordinates are in the range [0,1].
//		if (0 <= a_barycentric.x() && a_barycentric.x() <= 1 &&
//		    0 <= a_barycentric.y() && a_barycentric.y() <= 1 &&
//			0 <= a_barycentric.z() && a_barycentric.z() <= 1) {
//
//			Eigen::Vector3d a_projected = a_barycentric.x() * va + a_barycentric.y() * vb + a_barycentric.z() * vc;
//
//			// The point lies cleanly, fully inside the triangle!
//			// Return along with the face index.
//			return ConvexHullPoint{face_index, a_projected};
//
//		} else {
//
//			// Find the next face to step to.
//			size_t new_face;
//
//			/*
//			 * For a point in barycentric coordinates (a,b,c), if a < 0, then the point lies outside the triangle,
//			 * on the opposite of the plane through BC and the origin. Same for b and c.
//			 *
//			 * While barycentric coordinate components can also be > 1, this always coincides with another being < 0.
//			 *
//			 * So, we simply check which component is negative to know which edge to cross over.
//			 */
//
//			if (a_barycentric.x() < 0) {
//				new_face = f.neighbour_bc;
//			} else if (a_barycentric.y() < 0) {
//				new_face = f.neighbour_ca;
//			} else if (a_barycentric.z() < 0) {
//				new_face = f.neighbour_ab;
//			} else {
//				throw std::runtime_error("Projection outside of face or face neighbours.");
//			}
//
//			if (new_face == last_face) {
//
//				std::cout << "Barycentric: " << a_barycentric.transpose() << std::endl;
//
//				// We stepped back to the previous face. The point is on the edge.
//				//
//				// Since we *did* enter this triangle, we know that we're beyond the plane perpendicular to the facet and containing the edge,
//				// so we must be in the wedge defined by the two planes... Hence, the closest point must be on the edge.
//
//				// Capping the barycentric coordinates to the face should put the point there.
//
//				Eigen::Vector3d euclidean;
//
//				if (a_barycentric.x() > 1.0) {
//					euclidean = vertices[f.a];
//				} else if (a_barycentric.y() > 1.0) {
//					euclidean = vertices[f.b];
//				} else if (a_barycentric.z() > 1.0) {
//					euclidean = vertices[f.c];
//				} else if (a_barycentric.x() < 0.0) {
//
//					Eigen::ParametrizedLine<double, 3> bc(vb, vc-vb);
//
//					euclidean = bc.pointAt(std::clamp(projectionParameter(bc, a), 0.0, 1.0));
//
//				} else if (a_barycentric.y() < 0.0) {
//
//					Eigen::ParametrizedLine<double, 3> ca(vc, va-vc);
//
//					euclidean = ca.pointAt(std::clamp(projectionParameter(ca, a), 0.0, 1.0));
//
//				} else if (a_barycentric.z() < 0.0) {
//
//					Eigen::ParametrizedLine<double, 3> ab(va, vb-va);
//
//					euclidean = ab.pointAt(std::clamp(projectionParameter(ab, a), 0.0, 1.0));
//
//				} else {
//					throw std::runtime_error("Barycentric coordinates are weird.");
//				}
//
//				return ConvexHullPoint{face_index, euclidean};
//
//			} else {
//				last_face = new_face;
//			}
//
//		}
//
//	}

}

ConvexHullPoint ConvexHullShell::gaussian_sample_near_point(const ConvexHullPoint &near) const {

	const Facet &f = facets[near.face_id];

	ompl::RNG rng;

	Eigen::Vector3d a_rand =
			near.position + Eigen::Vector3d(rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1));

	return project(a_rand);

}

ConvexHullPoint ConvexHullShell::project(const moveit::core::RobotState &st) const {
	return project(st.getGlobalLinkTransform("base_link").translation());
}

ConvexHullPoint ConvexHullShell::project(const Apple &st) const {
	return project(st.center);
}

//#define DUMP_GEOGEBRA

std::vector<ConvexHullPoint>
ConvexHullShell::convex_hull_walk(const ConvexHullPoint &a, const ConvexHullPoint &b) const {

	std::set<size_t> visited_faces;

	const double epsilon = 10e-10;

	if (a.face_id == b.face_id) {
		return {a, b};
	} else {

		// A quick-and-dirty greedy approximation of the geodesic distance between the two points on the convex hull.
		// We start from point A, then "walk" along the surface of the convex hull, and then end up at point B.

		// First, we convert points A and B into Euclidean space.
		const Eigen::Vector3d a_euc = a.position;
		const Eigen::Vector3d b_euc = b.position;

		const Eigen::ParametrizedLine<double, 3> ideal_line(a_euc, (b_euc - a_euc).normalized());

#ifdef DUMP_GEOGEBRA
		std::cout << "A = (" << a_euc.x() << ", " << a_euc.y() << ", " << a_euc.z() << ")" << std::endl;
		std::cout << "B = (" << b_euc.x() << ", " << b_euc.y() << ", " << b_euc.z() << ")" << std::endl;
#endif

		ConvexHullPoint middle_proj = project(0.5 * (a_euc + b_euc));
		Eigen::Vector3d middle_normal = (vertices[facets[middle_proj.face_id].b] -
										 vertices[facets[middle_proj.face_id].a]).cross(
				vertices[facets[middle_proj.face_id].c] - vertices[facets[middle_proj.face_id].a]).normalized();
		Eigen::Vector3d middle_proj_euc = middle_proj.position - middle_normal;

#ifdef DUMP_GEOGEBRA
		std::cout << "M = (" << middle_proj_euc.x() << ", " << middle_proj_euc.y() << ", " << middle_proj_euc.z() << ")"
				  << std::endl;
#endif

		const Eigen::Vector3d normal = (a_euc - middle_proj_euc).cross(b_euc - middle_proj_euc).normalized();

		const Eigen::Hyperplane<double, 3> cutting_plane(normal, -a_euc.dot(normal));

		assert(cutting_plane.absDistance(a_euc) < 1e-6);
		assert(cutting_plane.absDistance(b_euc) < 1e-6);
		assert(cutting_plane.absDistance(middle_proj_euc) < 1e-6);

		// Current "walking" position, which will start at point A.
		std::vector<ConvexHullPoint> walk{a};

		while (walk.back().face_id != b.face_id && (walk.back().position - b.position).squaredNorm() > 1e-10) {

			bool is_first_step = walk.size() == 1;

			size_t current_face = walk.back().face_id;

			if (visited_faces.find(current_face) != visited_faces.end()) {

#ifdef DUMP_GEOGEBRA
				// Print the walk
				std::cout << "walk = Polyline({";

				for (size_t i = 0; i < walk.size(); ++i) {
					std::cout << "(" << walk[i].position.x() << ", " << walk[i].position.y() << ", "
							  << walk[i].position.z() << ")";
					if (i < walk.size() - 1) {
						std::cout << ", ";
					}
				}

				std::cout << "})" << std::endl << std::flush;
#endif

				throw std::runtime_error("Walked through face twice.");
			} else {
				visited_faces.insert(current_face);
			}


			// Create variables to be set later.
			size_t next_face;         // Which face to step to next.
			Eigen::Vector3d next_pt; // The point to step to next.

			// Look up the vertices of the triangle.
			auto va = vertices[facets[current_face].a];
			auto vb = vertices[facets[current_face].b];
			auto vc = vertices[facets[current_face].c];

#ifdef DUMP_GEOGEBRA
			std::cout << "face" << current_face << " = Polygon({(" << va.x() << ", " << va.y() << ", " << va.z() << "), (" << vb.x() << ", " << vb.y() << ", " << vb.z() << "), (" << vc.x() << ", " << vc.y() << ", " << vc.z() << ")}) " << std::endl;
#endif

			// For special cases, check if any are exactly pon the cutting plane.
			bool va_on_plane = cutting_plane.absDistance(va) < 1e-10;
			bool vb_on_plane = cutting_plane.absDistance(vb) < 1e-10;
			bool vc_on_plane = cutting_plane.absDistance(vc) < 1e-10;

			// Check if any whole edge of the triangle lies on the triangle, which is another special case.
			bool edge_ab_on_plane = va_on_plane && vb_on_plane;
			bool edge_bc_on_plane = vb_on_plane && vc_on_plane;
			bool edge_ca_on_plane = vc_on_plane && va_on_plane;

			// Construct parameterized lines that extend the edges of the triangles.
			Eigen::ParametrizedLine<double, 3> edge_ab(va, vb - va);
			Eigen::ParametrizedLine<double, 3> edge_bc(vb, vc - vb);
			Eigen::ParametrizedLine<double, 3> edge_ca(vc, va - vc);

			if (edge_ab_on_plane) {

				// As a default, we'll step to the AB-neighbour.
				next_face = facets[current_face].neighbour_ab;

				// Project our goal point onto the edge.
				double t = projectionParameter(edge_ab, b_euc);

				if (t - epsilon < 0.0) {
					// Goal point is on the extended edge, beyond vertex A.
					// Step to vertex A.
					next_pt = va;
				} else if (t + epsilon > 1.0) {
					// Goal point is on the extended edge, beyond vertex B.
					// Step to vertex B.
					next_pt = vb;
				} else {
					// Goal point is on the extended edge, somewhere between vertex A and B.
					// Step to the point on the extended edge.
					next_pt = edge_ab.pointAt(t);

					// As a matter of fact, this point corresponds to the goal point in euclidean coordinates.
					// Algorithm should terminate next iteration as we enter the goal facet.
				}

			} else if (edge_bc_on_plane) {
				// Same logic, but for BC
				next_face = facets[current_face].neighbour_bc;

				double t = projectionParameter(edge_bc, b_euc);

				if (t - epsilon < 0.0) {
					next_pt = vb;
				} else if (t + epsilon > 1.0) {
					next_pt = vc;
				} else {
					next_pt = edge_bc.pointAt(t);
				}
			} else if (edge_ca_on_plane) {
				// And for AC...
				next_face = facets[current_face].neighbour_ca;

				double t = projectionParameter(edge_ca, b_euc);

				if (t - epsilon < 0.0) {
					next_pt = va;
				} else if (t + epsilon > 1.0) {
					next_pt = vc;
				} else {
					next_pt = edge_ca.pointAt(t);
				}
			} else {

				// Ok, so no edges in the plane. That makes intersection calculations meaningful.
				double t_ab = edge_ab.intersection(cutting_plane);
				double t_bc = edge_bc.intersection(cutting_plane);
				double t_ca = edge_ca.intersection(cutting_plane);

				// Check if the plane intersects the edges on their interior
				bool intersects_ab_strict = 0 + epsilon < t_ab && t_ab + epsilon < 1.0;
				bool intersects_bc_strict = 0 + epsilon < t_bc && t_bc + epsilon < 1.0;
				bool intersects_ca_strict = 0 + epsilon < t_ca && t_ca + epsilon < 1.0;

				if (is_first_step) {
					// This is the first step on the walk. We don't have to worry about stepping back as much as we should worry about departing in the right direction.

					if (intersects_ab_strict || intersects_bc_strict || intersects_ca_strict) {
						// We're actually, properly intersecting one of the edges.
						// Just pick whichever intersection is closest to B.

						double d_min = INFINITY;

						if (intersects_ab_strict) {
							d_min = std::min(d_min, (b_euc - edge_ab.pointAt(t_ab)).squaredNorm());
							next_face = facets[current_face].neighbour_ab;
							next_pt = edge_ab.pointAt(t_ab);
						}

						if (intersects_bc_strict) {
							d_min = std::min(d_min, (b_euc - edge_bc.pointAt(t_bc)).squaredNorm());
							next_face = facets[current_face].neighbour_bc;
							next_pt = edge_bc.pointAt(t_bc);
						}

						if (intersects_ca_strict) {
							d_min = std::min(d_min, (b_euc - edge_ca.pointAt(t_ca)).squaredNorm());
							next_face = facets[current_face].neighbour_ca;
							next_pt = edge_ca.pointAt(t_ca);
						}

						assert(d_min < INFINITY);

					} else {

						// We must be at a corner. Pick the edge whose middle point is closest.

						double d_min = INFINITY;

						if (va_on_plane) {
							next_pt = va;
							if ((b_euc - edge_ab.pointAt(0.5)).squaredNorm() < d_min) {
								d_min = (b_euc - edge_ab.pointAt(0.5)).squaredNorm();
								next_face = facets[current_face].neighbour_ab;
							}
							if ((b_euc - edge_ca.pointAt(0.5)).squaredNorm() < d_min) {
								d_min = (b_euc - edge_ca.pointAt(0.5)).squaredNorm();
								next_face = facets[current_face].neighbour_ca;
							}
						} else if (vb_on_plane) {
							next_pt = vb;
							if ((b_euc - edge_bc.pointAt(0.5)).squaredNorm() < d_min) {
								d_min = (b_euc - edge_bc.pointAt(0.5)).squaredNorm();
								next_face = facets[current_face].neighbour_bc;
							}
							if ((b_euc - edge_ab.pointAt(0.5)).squaredNorm() < d_min) {
								d_min = (b_euc - edge_ab.pointAt(0.5)).squaredNorm();
								next_face = facets[current_face].neighbour_ab;
							}
						} else if (vc_on_plane) {
							next_pt = vc;
							if ((b_euc - edge_ca.pointAt(0.5)).squaredNorm() < d_min) {
								d_min = (b_euc - edge_ca.pointAt(0.5)).squaredNorm();
								next_face = facets[current_face].neighbour_ca;
							}
							if ((b_euc - edge_bc.pointAt(0.5)).squaredNorm() < d_min) {
								d_min = (b_euc - edge_bc.pointAt(0.5)).squaredNorm();
								next_face = facets[current_face].neighbour_bc;
							}
						} else {
							throw std::runtime_error("Not cleanly intersecting an edge, but not at a corner either.");
						}

						assert(d_min < INFINITY);


					}

				} else {

					// Also, there will be at least two intersections between the edges and the plane.
					// We need to pick the one where we don't step backward.
					bool ab_goes_back = facets[current_face].neighbour_ab == walk[walk.size() - 3].face_id;
					bool bc_goes_back = facets[current_face].neighbour_bc == walk[walk.size() - 3].face_id;
					bool ca_goes_back = facets[current_face].neighbour_ca == walk[walk.size() - 3].face_id;

					if (intersects_ab_strict && !ab_goes_back) {
						next_face = facets[current_face].neighbour_ab;
						next_pt = edge_ab.pointAt(t_ab);
					} else if (intersects_bc_strict && !bc_goes_back) {
						next_face = facets[current_face].neighbour_bc;
						next_pt = edge_bc.pointAt(t_bc);
					} else if (intersects_ca_strict && !ca_goes_back) {
						next_face = facets[current_face].neighbour_ca;
						next_pt = edge_ca.pointAt(t_ca);
					} else {
						// Egads! We're in a corner. Which one?

						if (va_on_plane) {
							// We're at vertex A.

							next_pt = va; // We'll stay there...

							// And "walk" around A in whatever direction is not where we just came from.
							if (!ab_goes_back) {
								next_face = facets[current_face].neighbour_ab;
							} else if (!ca_goes_back) {
								next_face = facets[current_face].neighbour_ca;
							} else {
								// If we get here, the mesh is broken.
								throw std::runtime_error("Two edges of a facet lead to the same facet!");
							}
						} else if (vb_on_plane) {
							// Same for vertex B.

							next_pt = vb;

							if (!bc_goes_back) {
								next_face = facets[current_face].neighbour_bc;
							} else if (!ab_goes_back) {
								next_face = facets[current_face].neighbour_ab;
							} else {
								throw std::runtime_error("Two edges of a facet lead to the same facet!");
							}
						} else if (vc_on_plane) {
							// Same for vertex C.

							next_pt = vc;

							if (!ca_goes_back) {
								next_face = facets[current_face].neighbour_ca;
							} else if (!bc_goes_back) {
								next_face = facets[current_face].neighbour_bc;
							} else {
								throw std::runtime_error("Two edges of a facet lead to the same facet!");
							}
						} else {

#ifdef DUMP_GEOGEBRA
							std::cout << "walk = PolyLine({";

							for (size_t i = 0; i < walk.size(); i++) {
								std::cout << "(" << walk[i].position.x() << ", " << walk[i].position.y() << ", " << walk[i].position.z() << ")";
								if (i < walk.size() - 1) {
									std::cout << ", ";
								}
							}

							std::cout << "})" << std::endl;
#endif

							// If we get here, the mesh is broken.
							throw std::runtime_error("Two edges of a facet lead to the same facet!");
						}
					}
				}
			}

			walk.push_back(ConvexHullPoint{current_face, next_pt});
			walk.push_back(ConvexHullPoint{next_face, next_pt});
		}

		walk.push_back(b);

		return walk;
	}
}

size_t ConvexHullShell::num_facets() const {
	return facets.size();
}

const ConvexHullShell::Facet &ConvexHullShell::facet(size_t i) const {
	return facets[i];
}

const Eigen::Vector3d &ConvexHullShell::vertex(size_t i) const {
	return vertices[i];
}

std::array<Eigen::Vector3d, 3> ConvexHullShell::facet_vertices(size_t facet_i) const {
	const Facet &f = facet(facet_i);

	return {vertex(f.a), vertex(f.b), vertex(f.c)};
}


double ConvexHullShell::facet_signed_distance(const Eigen::Vector3d ptr, size_t facet_index) const {
	const auto &[a, b, c] = facet_vertices(facet_index);

	Eigen::Vector3d ab = b - a;
	Eigen::Vector3d ac = c - a;

	Eigen::Vector3d n = ab.cross(ac);

	return n.dot(ptr - a);
}

double ConvexHullShell::signed_distance(const Eigen::Vector3d &pt) const {
	double d = -INFINITY;

	for (size_t i = 0; i < facets.size(); i++) {
		d = std::max(d, facet_signed_distance(pt, i));
	}

	return d;
}

Eigen::Vector3d ConvexHullShell::facet_normal(size_t i) const {
	const auto &[a, b, c] = facet_vertices(i);

	return (b - a).cross(c - a).normalized();
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

	return std::make_shared<OMPLSphereShellWrapper<ConvexHullPoint>>(std::make_shared<ConvexHullShell>(convexHull(
			leaf_vertices)), si);

}


Json::Value ConvexHullShellBuilder::parameters() const {
	Json::Value params;

	params["type"] = "convex_hull";

	return params;
}

bool ConvexHullShell::NNGNATEntry::operator==(const ConvexHullShell::NNGNATEntry &rhs) const {
	return face_index == rhs.face_index && at == rhs.at;
}

bool ConvexHullShell::NNGNATEntry::operator!=(const ConvexHullShell::NNGNATEntry &rhs) const {
	return !(rhs == *this);
}
