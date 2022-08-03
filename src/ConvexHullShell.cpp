
#include "ConvexHullShell.h"
#include "convex_hull.h"
#include "math.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/numeric/accumulate.hpp>


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
 * @param va			The first vertex of the triangle.
 * @param vb			The second vertex of the triangle.
 * @param vc			The third vertex of the triangle.
 * @return				The barycentric coordinates of the point (alpha, beta, gamma).
 */
Eigen::Vector3d pointInTriangle(const Eigen::Vector3d& qp,
								const Eigen::Vector3d& va,
								const Eigen::Vector3d& vb,
								const Eigen::Vector3d& vc)
{
	// u=P2−P1
	Eigen::Vector3d u = vb - va;
	// v=P3−P1
	Eigen::Vector3d v = vc - va;
	// n=u×v
	Eigen::Vector3d n = u.cross(v);
	// w=P−P1
	Eigen::Vector3d w = qp - va;
	// Barycentric coordinates of the projection P′of P onto T:
	// γ=[(u×w)⋅n]/n²
	double gamma = u.cross(w).dot(n) / n.dot(n);
	// β=[(w×v)⋅n]/n²
	double beta = w.cross(v).dot(n) / n.dot(n);
	double alpha = 1 - gamma - beta;

	return {
			alpha, beta, gamma
	};
}


moveit::core::RobotState
ConvexHullShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const ConvexHullPoint &a) const {

	const Facet& facet = facets[a.face_id];

	Eigen::Vector3d desired_ee_pos = a.position;

	Eigen::Vector3d normal = (vertices[facet.b] - vertices[facet.a]).cross(vertices[facet.c] - vertices[facet.a]).normalized();

	Eigen::Vector3d required_facing = normal;

	return robotStateFromFacing(drone, desired_ee_pos, required_facing);

}

std::vector<moveit::core::RobotState> ConvexHullShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																	 const ConvexHullPoint &a,
																	 const ConvexHullPoint &b) const {

	std::vector<moveit::core::RobotState> path;

	for (const ConvexHullPoint& pt : convex_hull_walk(a, b)) {
		path.push_back(state_on_shell(drone, pt));
	}

	return path;

}




double ConvexHullShell::predict_path_length(const ConvexHullPoint &a, const ConvexHullPoint &b) const {

	// I don't like that I'm allocating here... We'll see how bad the bottleneck is.
	auto walk = convex_hull_walk(a,b);

	double length = 0;

	for (size_t i = 1; i < walk.size(); i++) {
		length += (walk[i].position - walk[i-1].position).norm();
	}

	return length;
}


size_t ConvexHullShell::guess_closest_face(const Eigen::Vector3d &a) const {
	return facet_index.nearest(NNGNATEntry{SIZE_MAX, a}).face_index;
}

struct UnorderedEdge {
	size_t a, b;

	bool operator==(const UnorderedEdge &other) const {
		return a == other.a && b == other.b || a == other.b && b == other.a;
	}
};

template<> struct std::hash<UnorderedEdge> {
	size_t operator()(const UnorderedEdge &e) const {
		return std::hash<size_t>()(e.a) ^ std::hash<size_t>()(e.b);
	}
};

ConvexHullShell::ConvexHullShell(const shape_msgs::msg::Mesh &mesh) {

	this->vertices = mesh.vertices | ranges::views::transform([](const geometry_msgs::msg::Point &p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
	}) | ranges::to_vector;

	// Find a point inside of the mesh.
	Eigen::Vector3d com = ranges::accumulate(vertices,Eigen::Vector3d(0.0,0.0,0.0)) / (double) vertices.size();

	this->facets = mesh.triangles | ranges::views::transform([&](const shape_msgs::msg::MeshTriangle &t) {

		Facet f = {t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2], 0, 0, 0};

		Eigen::Vector3d normal = (vertices[f.b] - vertices[f.a]).cross(vertices[f.c] - vertices[f.a]).normalized();

		// Ensure winding order puts the normal pointing out from the mesh
		if (normal.dot(com - vertices[f.a]) < 0) {
			std::swap(f.b, f.c);
		}

		return f;

	}) | ranges::to_vector;

	match_faces();
	init_gnat();

}

void ConvexHullShell::init_gnat() {
	facet_index.setDistanceFunction([](const NNGNATEntry &a, const NNGNATEntry &b) {
		return (a.at - b.at).norm();
	});

	for (const auto &[i,f] : this->facets | ranges::views::enumerate) {
		facet_index.add(NNGNATEntry{i, (this->vertices[f.a] + this->vertices[f.b] + this->vertices[f.c]) / 3.0});
	}
}

void ConvexHullShell::match_faces() {

	struct FaceEdge {
		size_t face_index;
		size_t* edge_field;
	};

	std::unordered_map<UnorderedEdge, FaceEdge> edge_to_face;

	for (size_t i = 0; i < facets.size(); i++) {

		{
			UnorderedEdge ab = {facets[i].a, facets[i].b};

			if (edge_to_face.find(ab) == edge_to_face.end()) {
				edge_to_face[ab] = {i, &facets[i].neighbour_ab};
			} else {
				facets[i].neighbour_ab = edge_to_face[ab].face_index;
				*edge_to_face[ab].edge_field = i;
				edge_to_face.erase(ab);
			}
		}

		{
			UnorderedEdge bc = {facets[i].b, facets[i].c};
			if (edge_to_face.find(bc) == edge_to_face.end()) {
				edge_to_face[bc] = {i, &facets[i].neighbour_bc};
			} else {
				facets[i].neighbour_bc = edge_to_face[bc].face_index;
				*edge_to_face[bc].edge_field = i;
				edge_to_face.erase(bc);
			}
		}

		{
			UnorderedEdge ca = {facets[i].c, facets[i].a};
			if (edge_to_face.find(ca) == edge_to_face.end()) {
				edge_to_face[ca] = {i, &facets[i].neighbour_ca};
			} else {
				facets[i].neighbour_ca = edge_to_face[ca].face_index;
				*edge_to_face[ca].edge_field = i;
				edge_to_face.erase(ca);
			}
		}

	}
}

ConvexHullPoint ConvexHullShell::project(const Eigen::Vector3d &a) const {

	size_t face_index = guess_closest_face(a);

	size_t last_face = SIZE_MAX;

	size_t iters = 0;

	while (true) {

		iters += 1;

		const Facet &f = facets[face_index];

		Eigen::Vector3d a_barycentric = pointInTriangle(a, vertices[f.a], vertices[f.b], vertices[f.c]);

		Eigen::Vector3d a_proj_euclidean = vertices[f.a] * a_barycentric.x() + vertices[f.b] * a_barycentric.y() + vertices[f.c] * a_barycentric.z();
		Eigen::Vector3d normal = (vertices[f.b] - vertices[f.a]).cross(vertices[f.c] - vertices[f.a]).normalized();

		if (0 <= a_barycentric.x() && a_barycentric.x() <= 1 && 0 <= a_barycentric.y() && a_barycentric.y() <= 1 && 0 <= a_barycentric.z() && a_barycentric.z() <= 1) {
			return ConvexHullPoint{face_index, a_proj_euclidean};
		} else {

			size_t new_face;

			if (a_barycentric.x() < 0) {
				new_face = f.neighbour_bc;
			} else if (a_barycentric.y() < 0) {
				new_face = f.neighbour_ca;
			} else if (a_barycentric.z() < 0) {
				new_face = f.neighbour_ab;
			} else {
				throw std::runtime_error("Projection outside of face or face neighbours.");
			}

			if (new_face == last_face) {
				// We stepped back to the previous face. The point is on the edge.
				// Capping the barycentric coordinates to the face should put the point there.

				// TODO: Verify that this actually works?

				Eigen::Vector3d capped_barycentric(
						std::clamp(a_barycentric.x(), 0.0, 1.0),
						std::clamp(a_barycentric.y(), 0.0, 1.0),
						std::clamp(a_barycentric.z(), 0.0, 1.0)
						);

				capped_barycentric /= capped_barycentric.x() + capped_barycentric.y() + capped_barycentric.z();

				Eigen::Vector3d euclidean = vertices[f.a] * capped_barycentric.x() + vertices[f.b] * capped_barycentric.y() + vertices[f.c] * capped_barycentric.z();

				return ConvexHullPoint{face_index, euclidean};

			} else {
				last_face = new_face;
			}

		}

	}

}

ConvexHullPoint ConvexHullShell::gaussian_sample_near_point(const ConvexHullPoint &near) const {

	const Facet &f = facets[near.face_id];

	ompl::RNG rng;

	Eigen::Vector3d a_rand = near.position + Eigen::Vector3d(rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1));

	return project(a_rand);

}

ConvexHullPoint ConvexHullShell::project(const moveit::core::RobotState &st) const {
	return project(st.getGlobalLinkTransform("base_link").translation());
}

ConvexHullPoint ConvexHullShell::project(const Apple &st) const {
	return project(st.center);
}

std::vector<ConvexHullPoint> ConvexHullShell::convex_hull_walk(const ConvexHullPoint &a, const ConvexHullPoint &b) const {
	// A quick-and-dirty greedy approximation of the geodesic distance between the two points on the convex hull.
	// We start from point A, then "walk" along the surface of the convex hull, and then end up at point B.

	// First, we convert points A and B into Euclidean space.
	const Eigen::Vector3d a_euc = a.position;
	const Eigen::Vector3d b_euc = b.position;

	std::cout << "From (" << a_euc.x() << ", " << a_euc.y() << ", " << a_euc.z() << ") to (" << b_euc.x() << ", " << b_euc.y() << ", " << b_euc.z() << ")" << std::endl;

	Eigen::ParametrizedLine<double, 3> ideal_line(a_euc, (b_euc - a_euc).normalized());
double ideal_t = 0.0;

	// Current "walking" position, which will start at point A.
	std::vector<ConvexHullPoint> walk {a};

	while (walk.back().face_id != b.face_id) {

		size_t current_face = walk.back().face_id;

		// Look up the vertices of the triangle.
		auto va = vertices[facets[current_face].a];
		auto vb = vertices[facets[current_face].b];
		auto vc = vertices[facets[current_face].c];

		std::cout << "Face : (" << va.x() << ", " << va.y() << ", " << va.z() << "), (" << vb.x() << ", " << vb.y() << ", " << vb.z() << "), (" << vc.x() << ", " << vc.y() << ", " << vc.z() << ")" << std::endl;

		// Construct parameterized lines that extend the edges of the triangles.
		Eigen::ParametrizedLine<double, 3> edge_ab(va, vb - va);
		Eigen::ParametrizedLine<double, 3> edge_bc(vb, vc - vb);
		Eigen::ParametrizedLine<double, 3> edge_ca(vc, va - vc);

		// Find the closest approaches (expressed as parameters for the origin-ray form of the lines)
		auto [t_ab, s_ab] = closest_point_on_line(ideal_line, edge_ab);
		auto [t_bc, s_bc] = closest_point_on_line(ideal_line, edge_bc);
		auto [t_ca, s_ca] = closest_point_on_line(ideal_line, edge_ca);

		s_ab = std::clamp(s_ab, 0.0, 1.0);
		s_bc = std::clamp(s_bc, 0.0, 1.0);
		s_ca = std::clamp(s_ca, 0.0, 1.0);

		Eigen::Vector3d pt_ab = edge_ab.pointAt(s_ab);
		Eigen::Vector3d pt_bc = edge_bc.pointAt(s_bc);
		Eigen::Vector3d pt_ca = edge_ca.pointAt(s_ca);

		std::cout << "Closest points: (" << pt_ab.x() << ", " << pt_ab.y() << ", " << pt_ab.z() << "), (" << pt_bc.x() << ", " << pt_bc.y() << ", " << pt_bc.z() << "), (" << pt_ca.x() << ", " << pt_ca.y() << ", " << pt_ca.z() << ")" << std::endl;

		t_ab = ideal_line.direction().dot(pt_ab - ideal_line.origin());
		t_bc = ideal_line.direction().dot(pt_bc - ideal_line.origin());
		t_ca = ideal_line.direction().dot(pt_ca - ideal_line.origin());

		// Find which of the three point gets us the furthest along the idealized line.
		size_t next_face = SIZE_MAX;
		Eigen::Vector3d next_point;

		// We demand 0 < t_ab so that we don't go backwards,
		// and that 0 <= s_ab <= 1 so make sure we're actually passing over/under the edge
		if (t_ab > ideal_t) {
			ideal_t = t_ab;
			next_face = facets[current_face].neighbour_ab;
			next_point = edge_ab.pointAt(s_ab);

			std::cout << "s: " << s_ab << ", t: " << t_ab << std::endl;
		}

		if (t_bc > ideal_t) {
			ideal_t = t_bc;
			next_face = facets[current_face].neighbour_bc;
			next_point = edge_bc.pointAt(s_bc);

			std::cout << "s: " << s_bc << ", t: " << t_bc << std::endl;
		}

		if (t_ca > ideal_t) {
			ideal_t = t_ca;
			next_face = facets[current_face].neighbour_ca;
			next_point = edge_ca.pointAt(s_ca);

			std::cout << "s: " << s_ca << ", t: " << t_ca << std::endl;
		}

		// Make sure we hit at least one of the cases (which should always happen since we start on a point on the triangle)
		assert(next_face != SIZE_MAX);

		std::cout << "Pt: " << next_point.transpose() << std::endl;

		// We add a point for exiting this face...
		walk.push_back({walk.back().face_id, next_point});
		// ...and entering the next face, ensuring that interpolation is as easy as possible.
		walk.push_back({next_face, next_point});

	}

	std::cout << "Got there!" << std::endl;

	walk.push_back(b);

	return walk;
}

std::vector<geometry_msgs::msg::Point>
extract_leaf_vertices(const AppleTreePlanningScene &scene_info)  {
	std::vector<geometry_msgs::msg::Point> mesh_points;
	for (const auto& col : scene_info.scene_msg.world.collision_objects) {
		if (col.id == "leaves") {
			for (const auto& mesh : col.meshes) {
				for (auto v : mesh.vertices) {
					mesh_points.push_back(v);
				}
			}
		}
	}
	return mesh_points;
}

std::shared_ptr<OMPLSphereShellWrapper<ConvexHullPoint>> ConvexHullShellBuilder::buildShell(
		const AppleTreePlanningScene &scene_info,
		const ompl::base::SpaceInformationPtr &si) const {

	auto leaf_vertices = extract_leaf_vertices(scene_info);

	return std::make_shared<OMPLSphereShellWrapper<ConvexHullPoint>>(
			std::make_shared<ConvexHullShell>(
					convexHull(leaf_vertices)),si);

}




Json::Value ConvexHullShellBuilder::parameters() const {
	throw std::runtime_error("Not implemented");
}

bool ConvexHullShell::NNGNATEntry::operator==(const ConvexHullShell::NNGNATEntry &rhs) const {
	return face_index == rhs.face_index && at == rhs.at;
}

bool ConvexHullShell::NNGNATEntry::operator!=(const ConvexHullShell::NNGNATEntry &rhs) const {
	return !(rhs == *this);
}
