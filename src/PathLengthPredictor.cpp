//
// Created by werner on 25-3-23.
//

#include "PathLengthPredictor.h"
#include "utilities/delaunay.h"
#include "shell_space/DendriticConvexHullShell.h"

double EuclideanDistancePredictor::predict_path_length(const Apple &point1, const Apple &point2) {
	return (point1.center - point2.center).norm();
}

GreatCircleDistancePredictor::GreatCircleDistancePredictor(bodies::BoundingSphere enclosing_sphere) : enclosing_sphere(
		std::move(enclosing_sphere)) {
}

double GreatCircleDistancePredictor::predict_path_length(const Apple &point1, const Apple &point2) {

	Eigen::Vector3d ray1 = point1.center - enclosing_sphere.center;
	Eigen::Vector3d ray2 = point2.center - enclosing_sphere.center;

	double cos = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

	return enclosing_sphere.radius * std::acos(std::clamp(cos, -1.0, 1.0));

}

[[maybe_unused]] GreatCircleDistancePredictor
GreatCircleDistancePredictor::mec_around_leaves(const AppleTreePlanningScene &scene_info) {
	return GreatCircleDistancePredictor(utilities::compute_enclosing_sphere_around_leaves(*scene_info.scene_msg, 0.0));
}

const bodies::BoundingSphere &GreatCircleDistancePredictor::getEnclosingSphere() const {
	return enclosing_sphere;
}

std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(EuclideanDistancePredictor predictor) {

	Json::Value json;
	json["name"] = "EuclideanDistancePredictor";

	return std::make_pair(json, std::make_shared<EuclideanDistancePredictor>(predictor));

}

std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(GreatCircleDistancePredictor predictor) {

	Json::Value json;
	json["name"] = "GreatCircleDistancePredictor";
	json["enclosing_sphere"]["center"]["x"] = predictor.getEnclosingSphere().center.x();
	json["enclosing_sphere"]["center"]["y"] = predictor.getEnclosingSphere().center.y();
	json["enclosing_sphere"]["center"]["z"] = predictor.getEnclosingSphere().center.z();
	json["enclosing_sphere"]["radius"] = predictor.getEnclosingSphere().radius;

	return std::make_pair(json, std::make_shared<GreatCircleDistancePredictor>(predictor));
}

//std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>>
//pairWithJson(DendriticConvexHullDistancePredictor predictor) {
//
//	Json::Value json;
//	json["name"] = "DendriticConvexHullDistancePredictor";
//
//	return std::make_pair(json, std::make_shared<DendriticConvexHullDistancePredictor>(predictor));
//
//}

std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>>
pairWithJson(CuttingPlaneConvexHullDistancePredictor predictor) {
	Json::Value json;
	json["type"] = "CuttingPlaneConvexHullDistancePredictor";
	return std::make_pair(json, std::make_shared<CuttingPlaneConvexHullDistancePredictor>(std::move(predictor)));
}

double CuttingPlaneConvexHullDistancePredictor::predict_path_length(const Apple &point1, const Apple &point2) {
	return enclosing_shell.path_length(enclosing_shell.path_from_to(enclosing_shell.nearest_point_on_shell(point1.center),
																	enclosing_shell.nearest_point_on_shell(point2.center)));
}

CuttingPlaneConvexHullDistancePredictor::CuttingPlaneConvexHullDistancePredictor(const CuttingPlaneConvexHullShell &enclosingShell)
		: enclosing_shell(enclosingShell) {
}

CuttingPlaneConvexHullDistancePredictor
CuttingPlaneConvexHullDistancePredictor::around_leaves(const AppleTreePlanningScene &scene_info) {

	return CuttingPlaneConvexHullDistancePredictor(CuttingPlaneConvexHullShell(convexHull(utilities::extract_leaf_vertices(
			scene_info)), 0.0, 0.0));

}
//
//CGALConvexHullDistancePredictor::CGALConvexHullDistancePredictor(const CGAL::Surface_mesh<CGAL::Epick::Point_3> &enclosingShell)
//		: enclosing_shell(enclosingShell, 0.0, 0.0) {
//
//
//}

double CGALConvexHullDistancePredictor::predict_path_length(const Apple &point1, const Apple &point2) {

	return enclosing_shell.path_length(enclosing_shell.path_from_to(enclosing_shell.nearest_point_on_shell(point1.center),
																	enclosing_shell.nearest_point_on_shell(point2.center)));
}

CGALConvexHullDistancePredictor::CGALConvexHullDistancePredictor(const shape_msgs::msg::Mesh &mesh) : enclosing_shell(convexHull(mesh.vertices), 0.0, 0.0) {

}



CGAL::Surface_mesh<CGAL::Epick::Point_3> extractConvexHullSurfaceMesh(const dendritic_convex_hull::Delaunay &dt) {
	CGAL::Surface_mesh<dendritic_convex_hull::Delaunay::Point> tmesh;

	std::unordered_map<dendritic_convex_hull::Delaunay::Point, CGAL::SM_Vertex_index> vertex_map;

	// Extract the surface triangles.
	for (auto itr = dt.finite_cells_begin(); itr != dt.finite_cells_end(); ++itr) {
		for (int i = 0; i < 4; ++i) {
			if (dt.is_infinite(itr->neighbor(i))) {

				auto triangle = utilities::facet_triangle(itr, i);

				for (int j = 0; j < 3; ++j) {
					const auto &vertex = triangle.vertex(j);
					if (vertex_map.find(vertex) == vertex_map.end()) {
						vertex_map[vertex] = tmesh.add_vertex(vertex);
					}
				}

				tmesh.add_face(vertex_map[triangle.vertex(0)],
							   vertex_map[triangle.vertex(1)],
							   vertex_map[triangle.vertex(2)]);


			}
		}
	}
	return tmesh;
}

DendriticConvexHullDistancePredictor::DendriticConvexHullDistancePredictor(const shape_msgs::msg::Mesh &mesh) : dt(
		utilities::generateDelaunayTriangulation(mesh)) {

	dendrites = dendritic_convex_hull::extract_dendrites(dendritic_convex_hull::generate_parentage(dt), dt);

	tmesh = extractConvexHullSurfaceMesh(dt);

	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(tree);

}

std::shared_ptr<dendritic_convex_hull::DendriteNode>
find_closest_node(const std::vector<std::shared_ptr<dendritic_convex_hull::DendriteNode>> &dendrites,
				  const Apple &a1) {// Among the dendrites, find one with a node that is closest to the apple.
	std::shared_ptr<dendritic_convex_hull::DendriteNode> closest_node = nullptr;
	double closest_distance = std::numeric_limits<double>::max();

	std::vector<std::shared_ptr<dendritic_convex_hull::DendriteNode>> to_search_queue = dendrites;

	while (!to_search_queue.empty()) {

		auto node = to_search_queue.back();
		to_search_queue.pop_back();

		double distance = (node->position - a1.center).norm();
		if (distance < closest_distance) {
			closest_distance = distance;
			closest_node = node;
		}

		for (auto &child: node->children) {
			to_search_queue.push_back(child);
		}

	}

	return closest_node;
}

std::vector<Eigen::Vector3d> trace_dendrite(std::shared_ptr<dendritic_convex_hull::DendriteNode> closest_node1) {
	std::vector<Eigen::Vector3d> path;

	do {
		path.push_back(closest_node1->position);
		closest_node1 = closest_node1->parent.lock();
	} while (closest_node1 != nullptr);

	return path;
}


double DendriticConvexHullDistancePredictor::predict_path_length(const Apple &a1, const Apple &a2) {

	auto d1 = trace_dendrite(find_closest_node(dendrites, a1));
	auto d2 = trace_dendrite(find_closest_node(dendrites, a2));
	std::reverse(d2.begin(), d2.end());

	// Now, the path along the shell.
	auto chull = extractConvexHullSurfaceMesh(dt);

	Surface_mesh_shortest_path shortest_paths(tmesh);

	auto shell_1 = shortest_paths.locate(dendritic_convex_hull::Point(d1[0].x(), d1[0].y(), d1[0].z()), tree);
	auto shell_2 = shortest_paths.locate(dendritic_convex_hull::Point(d2[0].x(), d2[0].y(), d2[0].z()), tree);

	shortest_paths.add_source_point(shell_2);

	std::vector<dendritic_convex_hull::Point> shell_path;
	shortest_paths.shortest_path_points_to_source_points(shell_1.first, shell_1.second, std::back_inserter(shell_path));

	auto shell_path_points = shell_path | ranges::view::transform([](const auto &p) {
		return Eigen::Vector3d(p.x(), p.y(), p.z());
	}) | ranges::to_vector;

	std::vector<Eigen::Vector3d> total_path;

	total_path.push_back(a1.center);
	total_path.insert(total_path.end(), d1.begin(), d1.end());
	total_path.insert(total_path.end(), shell_path_points.begin(), shell_path_points.end());
	total_path.insert(total_path.end(), d2.begin(), d2.end());
	total_path.push_back(a2.center);

	double length = 0.0;
	for (int i = 1; i < total_path.size(); ++i) {
		length += (total_path[i] - total_path[i - 1]).norm();
	}

	return length;

}

