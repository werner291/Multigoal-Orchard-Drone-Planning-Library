#include <boost/asio.hpp>
#include <range/v3/all.hpp>

#include "../utilities/vtk.h"
#include "../visualization/compute_prm.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../utilities/enclosing_sphere.h"
#include "../utilities/convex_hull.h"
#include "../utilities/mesh_utils.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"
#include "../shell_space/CGALMeshShell.h"
#include "../visualization/path_visualization.h"

int main(int argc, char **argv) {

	std::vector<TreeMeshes> tree_models = loadRandomTreeModels(5, 500);

	SimplifiedOrchard orchard = makeSingleRowOrchard(tree_models);

	SimpleVtkViewer viewer;

	addSimplifiedOrchardToViewer(viewer, orchard);

	viewer.addMesh(createGroundPlane(100.0, 100.0), {0.5, 0.3, 0.1}, 1.0);

	auto leafVertices = utilities::extract_leaf_vertices(orchard);

	auto convex_hull = convexHull(leafVertices);

	convex_hull.triangles
			.erase(std::remove_if(convex_hull.triangles.begin(), convex_hull.triangles.end(), [&](auto face) {
				auto pta = convex_hull.vertices[face.vertex_indices[0]];
				auto ptb = convex_hull.vertices[face.vertex_indices[1]];
				auto ptc = convex_hull.vertices[face.vertex_indices[2]];

				bool neg_y = pta.y < 0.0 || ptb.y < 0.0 || ptc.y < 0.0;
				bool pos_y = pta.y > 0.0 || ptb.y > 0.0 || ptc.y > 0.0;

				bool all_below_z = pta.z < 1.0 || ptb.z < 1.0 || ptc.z < 1.0;

				return neg_y && pos_y && all_below_z;
			}), convex_hull.triangles.end());

	viewer.addMesh(convex_hull, {0.8, 0.8, 0.8}, 0.5);

	auto cutting_plane_chull_shell = std::make_shared<CGALMeshShell>(convex_hull, 1.0, 0.0);

	for (int i = 0; i < 100; ++i) {
		// Pick two trees at random, with replacement.
		std::pair<Eigen::Vector2d, TreeMeshes> tree_a = orchard.trees[1];
		std::pair<Eigen::Vector2d, TreeMeshes> tree_b = orchard.trees.back();

		// Find the centers.
		Eigen::Vector3d center_a =
				mesh_aabb(tree_a.second.fruit_meshes[rand() % tree_a.second.fruit_meshes.size()]).center() +
				Eigen::Vector3d(tree_a.first.x(), tree_a.first.y(), 0.0);
		Eigen::Vector3d center_b =
				mesh_aabb(tree_b.second.fruit_meshes[rand() % tree_b.second.fruit_meshes.size()]).center() +
				Eigen::Vector3d(tree_b.first.x(), tree_b.first.y(), 0.0);

		auto idealized_path = idealizedPathViaShell(*cutting_plane_chull_shell, center_a, center_b, 32);

		for (auto &pt: idealized_path) {
			std::cout << "(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
		}

		viewer.addStaticPolyline(idealized_path, {1.0, 0.0, 1.0});
	}

	viewer.start();

}

