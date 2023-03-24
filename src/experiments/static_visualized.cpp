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


std::shared_ptr<dch::DendriteNode> find_closest_node(const std::vector<std::shared_ptr<dch::DendriteNode>> &dendrites, const Apple &a1) {// Among the dendrites, find one with a node that is closest to the apple.
	std::shared_ptr<dch::DendriteNode> closest_node = nullptr;
	double closest_distance = std::numeric_limits<double>::max();

	std::vector<std::shared_ptr<dch::DendriteNode>> to_search_queue = dendrites;

	while (!to_search_queue.empty()) {

		auto node = to_search_queue.back();
		to_search_queue.pop_back();

		double distance = (node->position - a1.center).norm();
		if (distance < closest_distance) {
			closest_distance = distance;
			closest_node = node;
		}

		for (auto& child : node->children) {
			to_search_queue.push_back(child);
		}

	}

	return closest_node;
}

std::vector<Eigen::Vector3d> trace_dendrite(std::shared_ptr<dch::DendriteNode> closest_node1) {
	std::vector<Eigen::Vector3d> path;

	do {
		path.push_back(closest_node1->position);
		closest_node1 = closest_node1->parent.lock();
	} while (closest_node1 != nullptr);

	return path;
}

int main(int argc, char **argv) {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");
	auto apples = meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector;

	SimpleVtkViewer viewer;
	viewer.addMesh(meshes.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	dch::Delaunay dt = utilities::generateDelaunayTriangulation(meshes.trunk_mesh);

	auto dendrites = dch::extract_dendrites(dch::generate_parentage(dt), dt);

	// Pick a dendrite at random
//	auto dendrite = dendrites[std::rand() % dendrites.size()];
//
//	VtkLineSegmentsVisualization edges_viz(1.0,0.0,1.0);
//	edges_viz.updateLine(dch::extract_dendrite_edges(dendrite));
//
//	viewer.addActor(edges_viz.getActor());


//	std::vector<Delaunay::Cell_handle> big_cells;
//
//	for (auto itr = dt.finite_cells_begin(); itr != dt.finite_cells_end(); ++itr) {
//		Point circumcenter = itr->circumcenter();
//		double sphere_radius_sqr = squared_distance(circumcenter, itr->vertex(0)->point());
//		if (sphere_radius_sqr > 0.1 * 0.1) {
//			big_cells.push_back(itr);
//		}
//	}
//
	Apple a1 = apples[30];
	auto d1 = trace_dendrite(find_closest_node(dendrites, a1));

	Apple a2 = apples[101];
	auto d2 = trace_dendrite(find_closest_node(dendrites, a2));
	std::reverse(d2.begin(), d2.end());

	// Now, the path along the shell.

	CGAL::Surface_mesh<dch::Delaunay::Point> tmesh;

	std::unordered_map<dch::Delaunay::Point, CGAL::SM_Vertex_index> vertex_map;

	// Extract the surface triangles.
	for (auto itr = dt.finite_cells_begin(); itr != dt.finite_cells_end(); ++itr) {
		for (int i = 0; i < 4; ++i) {
			if (dt.is_infinite(itr->neighbor(i))) {

				std::array<dch::Delaunay::Point, 3> triangle;
				triangle[0] = itr->vertex((i + 1) % 4)->point();
				triangle[1] = itr->vertex((i + 2) % 4)->point();
				triangle[2] = itr->vertex((i + 3) % 4)->point();

				for (int j = 0; j < 3; ++j) {
					if (vertex_map.find(triangle[j]) == vertex_map.end()) {
						vertex_map[triangle[j]] = tmesh.add_vertex(triangle[j]);
					}
				}

				tmesh.add_face(vertex_map[triangle[0]], vertex_map[triangle[1]], vertex_map[triangle[2]]);

			}
		}
	}


	// We initialize the AABB tree such that we don't have to re-compute it every projection query.
	Surface_mesh_shortest_path shortest_paths(tmesh);

	CGAL::AABB_tree<AABBTraits> tree;
	shortest_paths.build_aabb_tree(tree);

	auto shell_1 = shortest_paths.locate(dch::Point(d1[0].x(), d1[0].y(), d1[0].z()), tree);
	auto shell_2 = shortest_paths.locate(dch::Point(d2[0].x(), d2[0].y(), d2[0].z()), tree);

	shortest_paths.add_source_point(shell_1);

	std::vector<dch::Point> shell_path;
	shortest_paths.shortest_path_points_to_source_points(shell_2.first, shell_2.second, std::back_inserter(shell_path));
	std::reverse(shell_path.begin(), shell_path.end());

	std::vector<Eigen::Vector3d> total_path;

	total_path.push_back(a1.center);
	total_path.insert(total_path.end(), d1.begin(), d1.end());

	for (auto& p : shell_path) {
		total_path.emplace_back(p.x(), p.y(), p.z());
	}

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


