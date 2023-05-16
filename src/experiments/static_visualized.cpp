#include <utility>
#include <boost/asio.hpp>
#include <range/v3/all.hpp>

#include "../utilities/vtk.h"
#include "../visualization/path_visualization.h"
#include "../visualization/compute_prm.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../utilities/enclosing_sphere.h"
#include "../utilities/convex_hull.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"
#include "../utilities/mesh_utils.h"
#include "../utilities/experiment_utils.h"
#include "../planners/shell_path_planner/Construction.h"
#include "../planner_allocators.h"
#include "../quickplan.h"
#include "../visualization/VtkRobotModel.h"
#include "../utilities/moveit.h"
#include "../RobotCameraTracker.h"

int main(int argc, char **argv) {

	std::vector<TreeMeshes> tree_models = loadRandomTreeModels(5, 500);

	SimplifiedOrchard orchard = makeSingleRowOrchard(tree_models);

	SimpleVtkViewer viewer;

	addSimplifiedOrchardToViewer(viewer, orchard);

	viewer.addMesh(createGroundPlane(100.0, 100.0), {0.5, 0.3, 0.1}, 1.0);

	auto leafVertices = utilities::extract_leaf_vertices(orchard);

	//	auto scene = createSceneFromSimplifiedOrchard(orchard);

	//
	//	addTreeMeshesToViewer(viewer, current_tree_models);
	//
	//    auto scene = createSceneFromTreeModels(current_tree_models);
	//
	//	// auto sphere_shell = paddedSphericalShellAroundLeaves(scene, 0.0);
	//    // auto sphere_actor = mkSphereShellActor(*sphere_shell);
	//    // viewer.addActor(sphere_actor);
	//
	auto convex_hull = convexHull(leafVertices);

	viewer.addMesh(convex_hull, {0.8, 0.8, 0.8}, 0.2);

	auto cutting_plane_chull_shell = std::make_shared<CuttingPlaneConvexHullShell>(convex_hull, 0.0, 0.0);
	//	// auto cgal_mesh_shell = std::make_shared<CGALMeshShell>(convex_hull, 0.0, 0.0);
	//	auto chull_actor = createColoredMeshActor(convex_hull, {0.8, 0.8, 0.8, 0.2}, true);
	//    viewer.addActor(chull_actor);
	//
	//	VtkLineSegmentsVisualization path_viz(1.0, 0.0, 1.0);
	//	viewer.addActor(path_viz.getActor());
	//
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

	//	viewer.addStaticPolyline(idealized_path, {1.0, 0.0, 1.0});

	auto robot = loadRobotModel();
	auto start_state = randomStateOutsideTree(robot, 42);
	auto planner_allocator = makeShellBasedPlanner<ConvexHullPoint>(cuttingPlaneChullShell);

	auto scene = createSceneFromSimplifiedOrchard(orchard);

	// shuffle the scene
	std::random_shuffle(scene.apples.begin(), scene.apples.end());

	// Get 500 apples.
	if (scene.apples.size() > 50)
		scene.apples.resize(50);

	// Set OMPL log level
	ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);

	RobotPath rpath_moveit = quickPlan(scene, start_state, planner_allocator);

	VtkRobotmodel robotModel(robot, start_state);
	viewer.addActorCollection(robotModel.getLinkActors());

	const auto trajectory = robotPathToConstantSpeedRobotTrajectory(rpath_moveit, 1.0);

	moveit::core::RobotState st(robot);

	RobotCameraTracker camera_tracker(viewer.viewerRenderer->GetActiveCamera(), start_state, mesh_aabb(convex_hull));

	double t = 0.0;

	viewer.addTimerCallback([&]() {
		setStateToTrajectoryPoint(st, t, trajectory);

		robotModel.applyState(st);
		camera_tracker.update(st);

		t += 0.04;

		if (t > 100) {
			viewer.stop();
		}
	});

	//	viewer.startRecording("orchard.ogv");

	viewer.start();

}

