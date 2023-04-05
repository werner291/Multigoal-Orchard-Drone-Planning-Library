#include <range/v3/all.hpp>
#include <boost/asio.hpp>

#include "../planning_scene_diff_message.h"
#include "../vtk/Viewer.h"

#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../planners/CachingDynamicPlanner.h"

#include "../vtk/SimpleVtkViewer.h"
#include "../utilities/delaunay.h"
#include "../shell_space/DendriticConvexHullShell.h"

#include <vtkProperty.h>
#include <CGAL/Delaunay_triangulation_3.h>

namespace dch = dendritic_convex_hull;


int main(int argc, char **argv) {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");
	auto apples = meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector;

	SimpleVtkViewer viewer;
	viewer.addMesh(meshes.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	dch::Delaunay dt = utilities::generateDelaunayTriangulation(meshes.trunk_mesh);

	auto dendrites = dch::extract_dendrites(dch::generate_parentage(dt), dt);

	Apple a1 = apples[30];
	auto d1 = trace_dendrite(find_closest_node(dendrites, a1));

	Apple a2 = apples[101];
	auto d2 = trace_dendrite(find_closest_node(dendrites, a2));
	std::reverse(d2.begin(), d2.end());

	// Now, the path along the shell.
	auto chull = extractConvexHullSurfaceMesh(dt);

	CGALMeshShell shell(chull, 0.0, 0.0);

	auto path = shell.path_from_to(
			shell.nearest_point_on_shell(d1.back()),
			shell.nearest_point_on_shell(d2.back())
			);

	std::vector<dch::Point> shell_path;

	auto shell_path_points = std::dynamic_pointer_cast<PiecewiseLinearPath<CGALMeshShellPoint>>(path)->points |
			ranges::views::transform([&](const auto& p) { return shell.surface_point(p); }) | ranges::to_vector;

	std::vector<Eigen::Vector3d> total_path;

	total_path.push_back(a1.center);
	total_path.insert(total_path.end(), d1.begin(), d1.end());
	total_path.insert(total_path.end(), shell_path_points.begin(), shell_path_points.end());
	total_path.insert(total_path.end(), d2.begin(), d2.end());
	total_path.push_back(a2.center);

	VtkPolyLineVisualization path_viz(1.0, 0.0, 0.0);

	path_viz.updateLine(total_path);

	viewer.addActor(path_viz.getActor());



//
//	auto c1 = utilities::closest_cell(Point(a1.center.x(), a2.center.y(), a2.center.z()), big_cells);
//	auto c2 = utilities::closest_cell(Point(a2.center.x(), a2.center.y(), a2.center.z()), big_cells);


	// find whichever cell is closest to the apple.


	viewer.start();

	exit(1);

	//
	//	// Convert the meshes to a planning scene message.
	//	const auto scene = AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(
	//			treeMeshesToMoveitSceneMsg(meshes))), .apples = meshes.fruit_meshes |
	//															ranges::views::transform(appleFromMesh) |
	//															ranges::to_vector};
	//
	//	// Load the robot model.
	//	const auto robot = loadRobotModel();
	//
	//	using namespace ranges;
	//
	//	const auto start_state = randomStateOutsideTree(robot, 0);
	//
	//	std::cout << "Starting planning with " << apple_discoverability.size() << " apples in total, of which "
	//			  << ranges::count(apple_discoverability, DISCOVERABLE) << " are discoverable." << std::endl;
	//
	//	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	//	// So, we just re-create the state space every time just to be safe.
	//	auto ss = omplStateSpaceForDrone(robot);
	//
	//	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	//	// we'll need to copy this for every thread
	//	auto si = loadSpaceInformation(ss, scene);
	//
	//	auto planner = dynamic_planner_initial_orbit(si);
	//
	//	auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(planner, si, ss);





}


