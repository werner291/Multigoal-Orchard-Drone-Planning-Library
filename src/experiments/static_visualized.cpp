
#include <utility>
#include <boost/asio.hpp>
#include <range/v3/all.hpp>

#include "../utilities/vtk.h"
#include "../visualization/path_visualization.h"
#include "../visualization/compute_prm.h"
#include "../visualization/SimpleVtkViewer.h"

int main(int argc, char **argv) {

	SimplifiedOrchard orchard {
		.trees= {
				{ Eigen::Vector2d(0.0,0.0), loadTreeMeshes("appletree") },
				{ Eigen::Vector2d(2.0,0.0), loadTreeMeshes("appletree2") },
		}
	};
//
////	auto current_tree_models = loadTreeMeshes("appletree");
//
	SimpleVtkViewer viewer;

	addSimplifiedOrchardToViewer(viewer, orchard);

//
//	addTreeMeshesToViewer(viewer, current_tree_models);
//
//    auto scene = createSceneFromTreeModels(current_tree_models);
//
//	// auto sphere_shell = paddedSphericalShellAroundLeaves(scene, 0.0);
//    // auto sphere_actor = mkSphereShellActor(*sphere_shell);
//    // viewer.addActor(sphere_actor);
//
//	auto convex_hull = convexHull(utilities::extract_leaf_vertices(scene));
//	auto cutting_plane_chull_shell = std::make_shared<CuttingPlaneConvexHullShell>(convex_hull, 0.0, 0.0);
//	// auto cgal_mesh_shell = std::make_shared<CGALMeshShell>(convex_hull, 0.0, 0.0);
//	auto chull_actor = createColoredMeshActor(convex_hull, {0.8, 0.8, 0.8, 0.2}, true);
//    viewer.addActor(chull_actor);
//
//	VtkLineSegmentsVisualization path_viz(1.0, 0.0, 1.0);
//	viewer.addActor(path_viz.getActor());
//
//	size_t from_apple = 0;
//    size_t to_apple = 42;
//
//	auto idealized_path = idealizedPathViaShell(*cutting_plane_chull_shell, scene.apples[from_apple].center, scene.apples[to_apple].center, 32);
//	viewer.addStaticPolyline(idealized_path, {1.0, 0.0, 1.0});
//
//    auto robot = loadRobotModel();
//    auto start_state = randomStateOutsideTree(robot, 42);
//	auto planner_allocator = makeShellBasedPlanner<ConvexHullPoint>(cuttingPlaneChullShell);
//
//	RobotPath rpath_moveit = quickPlan(scene, start_state, planner_allocator);
//
//	visualizeBaseEndEffectorLadderTrace(viewer, rpath_moveit);
//
	viewer.start();

}

