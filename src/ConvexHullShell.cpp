
#include "ConvexHullShell.h"
#include "convex_hull.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/numeric/accumulate.hpp>

moveit::core::RobotState
ConvexHullShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const ConvexHullPoint &a) const {

	const Facet& facet = facets[a.face_id];

	Eigen::Vector3d desired_ee_pos = to_euclidean(a, facet);

	Eigen::Vector3d normal = (vertices[facet.b] - vertices[facet.a]).cross(vertices[facet.c] - vertices[facet.a]).normalized();

	Eigen::Vector3d required_facing = normal;

	return robotStateFromFacing(drone, desired_ee_pos, required_facing);

}

Eigen::Vector3d ConvexHullShell::to_euclidean(const ConvexHullPoint &a, const ConvexHullShell::Facet &facet) const {
	Eigen::Vector3d desired_ee_pos = vertices[facet.a] * a.barycentric.x() + vertices[facet.b] * a.barycentric.y() + vertices[facet.c] * a.barycentric.z();
	return desired_ee_pos;
}

std::vector<moveit::core::RobotState> ConvexHullShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																	 const ConvexHullPoint &a,
																	 const ConvexHullPoint &b) const {
	throw std::runtime_error("Not implemented");
}

double ConvexHullShell::predict_path_length(const ConvexHullPoint &a, const ConvexHullPoint &b) const {

	throw std::runtime_error("Not implemented");

}

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

	for (size_t i = 0; i < facets.size(); i++) {
		// At least one edge must refer back to the given edge
		assert(
				facets[facets[i].neighbour_ab].neighbour_ab == i ||
				facets[facets[i].neighbour_ab].neighbour_bc == i ||
				facets[facets[i].neighbour_ab].neighbour_ca == i
		);

		assert(
				facets[facets[i].neighbour_bc].neighbour_ab == i ||
				facets[facets[i].neighbour_bc].neighbour_bc == i ||
				facets[facets[i].neighbour_bc].neighbour_ca == i
		);

		assert(
				facets[facets[i].neighbour_ca].neighbour_ab == i ||
				facets[facets[i].neighbour_ca].neighbour_bc == i ||
				facets[facets[i].neighbour_ca].neighbour_ca == i
		);

		for (size_t neighbour_index : {facets[i].neighbour_ab, facets[i].neighbour_bc, facets[i].neighbour_ca}) {
			size_t shared_vertices = 0;

			shared_vertices += (facets[i].a == facets[neighbour_index].a) + (facets[i].a == facets[neighbour_index].b) + (facets[i].a == facets[neighbour_index].c);
			shared_vertices += (facets[i].b == facets[neighbour_index].a) + (facets[i].b == facets[neighbour_index].b) + (facets[i].b == facets[neighbour_index].c);
			shared_vertices += (facets[i].c == facets[neighbour_index].a) + (facets[i].c == facets[neighbour_index].b) + (facets[i].c == facets[neighbour_index].c);

			assert(shared_vertices == 2);
		}

	}
}

ConvexHullPoint ConvexHullShell::project(const Eigen::Vector3d &a) const {

	std::cout << "Projecting " << a.transpose() << std::endl;

	size_t face_index = guess_closest_face(a);

	size_t iters = 0;

	while (true) {

		iters += 1;

		const Facet &f = facets[face_index];

		// TODO for tomorrow: If it tries to step back, conclude the point is on an edge?

		if (iters > 50) {
			std::cout << "On facet: " << face_index << " - "
				<< f.a << " " << f.b << " " << f.c
				<< "(" << vertices[f.a].x() << "," << vertices[f.a].y() << "," << vertices[f.a].z() << ")"
				<< "(" << vertices[f.b].x() << "," << vertices[f.b].y() << "," << vertices[f.b].z() << ")"
				<< "(" << vertices[f.c].x() << "," << vertices[f.c].y() << "," << vertices[f.c].z() << ")" << std::endl;

		}

		if (iters > 60) {
			exit(1);
		}

		Eigen::Vector3d a_barycentric = pointInTriangle(a, vertices[f.a], vertices[f.b], vertices[f.c]);

		Eigen::Vector3d a_proj_euclidean = vertices[f.a] * a_barycentric.x() + vertices[f.b] * a_barycentric.y() + vertices[f.c] * a_barycentric.z();
		Eigen::Vector3d normal = (vertices[f.b] - vertices[f.a]).cross(vertices[f.c] - vertices[f.a]).normalized();

		Eigen::Vector3d proj_ray = (a-a_proj_euclidean).normalized();

		assert(abs((proj_ray).dot(normal)) > 1.0 - 1.0e-6);

		if (0 <= a_barycentric.x() && a_barycentric.x() <= 1 && 0 <= a_barycentric.y() && a_barycentric.y() <= 1 && 0 <= a_barycentric.z() && a_barycentric.z() <= 1) {
			return ConvexHullPoint{face_index, a_barycentric};
		} else {

			if (a_barycentric.x() < 0) {
				face_index = f.neighbour_bc;
			} else if (a_barycentric.y() < 0) {
				face_index = f.neighbour_ca;
			} else if (a_barycentric.z() < 0) {
				face_index = f.neighbour_ab;
			} else {
				throw std::runtime_error("Projection outside of face or face neighbours.");
			}

		}

	}

}

ConvexHullPoint ConvexHullShell::gaussian_sample_near_point(const ConvexHullPoint &near) const {

	const Facet &f = facets[near.face_id];

	Eigen::Vector3d a_proj = vertices[f.a] * near.barycentric.x() + vertices[f.b] * near.barycentric.y() + vertices[f.c] * near.barycentric.z();

	ompl::RNG rng;

	Eigen::Vector3d a_rand = a_proj + Eigen::Vector3d(rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1));

	return project(a_rand);

}

ConvexHullPoint ConvexHullShell::project(const moveit::core::RobotState &st) const {
	return project(st.getGlobalLinkTransform("base_link").translation());
}

ConvexHullPoint ConvexHullShell::project(const Apple &st) const {
	return project(st.center);
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
