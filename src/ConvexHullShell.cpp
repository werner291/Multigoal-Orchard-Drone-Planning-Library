
#include "ConvexHullShell.h"
#include "convex_hull.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/iota.hpp>

moveit::core::RobotState
ConvexHullShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const {
	throw std::runtime_error("Not implemented");
}

std::vector<moveit::core::RobotState> ConvexHullShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																	 const Eigen::Vector3d &a,
																	 const Eigen::Vector3d &b) const {
	throw std::runtime_error("Not implemented");
}

double ConvexHullShell::predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {
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

	this->facets = mesh.triangles | ranges::views::transform([](const shape_msgs::msg::MeshTriangle &t) {
		return Facet{t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2], 0, 0, 0};
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
	std::unordered_map<UnorderedEdge, size_t> edge_to_face;
	for (size_t i = 0; i < facets.size(); i++) {

		UnorderedEdge ab = {facets[i].a, facets[i].b};
		if (edge_to_face.find(ab) == edge_to_face.end()) {
			edge_to_face[ab] = i;
		} else {
			facets[i].neighbour_ab = edge_to_face[ab];
			facets[edge_to_face[ab]].neighbour_ab = i;
		}

		UnorderedEdge bc = {facets[i].b, facets[i].c};
		if (edge_to_face.find(bc) == edge_to_face.end()) {
			edge_to_face[bc] = i;
		} else {
			facets[i].neighbour_bc = edge_to_face[bc];
			facets[edge_to_face[bc]].neighbour_bc = i;
		}

		UnorderedEdge ca = {facets[i].c, facets[i].a};
		if (edge_to_face.find(ca) == edge_to_face.end()) {
			edge_to_face[ca] = i;
		} else {
			facets[i].neighbour_ca = edge_to_face[ca];
			facets[edge_to_face[ca]].neighbour_ca = i;
		}

	}
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

std::shared_ptr<OMPLSphereShellWrapper> ConvexHullShellBuilder::buildShell(
		const AppleTreePlanningScene &scene_info,
		const ompl::base::SpaceInformationPtr &si) const {

	auto leaf_vertices = extract_leaf_vertices(scene_info);

	return std::make_shared<OMPLSphereShellWrapper>(std::make_shared<ConvexHullShell>(convexHull(leaf_vertices)),si);

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
