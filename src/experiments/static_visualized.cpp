#include <range/v3/all.hpp>
#include <boost/asio.hpp>

#include "../planning_scene_diff_message.h"
#include "../vtk/Viewer.h"

#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../planners/CachingDynamicPlanner.h"

#include "../vtk/SimpleVtkViewer.h"
#include "../utilities/delaunay.h"

#include <vtkProperty.h>
#include <CGAL/Delaunay_triangulation_3.h>


int main(int argc, char **argv) {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");
	auto apples = meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector;

	SimpleVtkViewer viewer;
	viewer.addMesh(meshes.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	using K = CGAL::Epick;
	using Delaunay = CGAL::Delaunay_triangulation_3<K>;
	using Point = Delaunay::Point;

	/*
	 * 1. Start by letting $T$ represent a [[Delaunay Tetrahedralization|delaunay tetrahedralization]] of the point set $O$.
	 *
	 * 2. Derive the [[Convex Hull|convex hull]] $C$ from $T$, which is simply the external surface of $T$.
	 *
	 * 3. For each cell $c$ in $T$ that has a circumradius greater than a specified $r > 0$, find the shortest path
	 *    between circumcenters (along edges of the [[Voronoi diagram]]) leading to $C$, traversing edges where
	 *    the associated facet in $T$ has a circumradius of at least $r$, until finding an edge that intersects $C$.
	 *    If the operation is successful, store the path as a _dendrite_.
	 *
	 * 4. Paths with overlapping edges can be combined to form trees of Voronoi edges. These trees are rooted at a point
	 *    on the convex hull.
	 */

	Delaunay dt = utilities::generateDelaunayTriangulation(meshes.trunk_mesh);

	struct Dendrite {
		Delaunay::Facet facet;
		std::array<K::FT, 3> barycentric;

	};

	struct CellVisit {

		Delaunay::Cell_handle cell;
		std::shared_ptr<CellVisit> parent;

	};

	// Extract the convex hull.
	std::vector<CellVisit> root_cells;
	std::queue<CellVisit> queue;

	for (auto cell_itr = dt.finite_cells_begin(); cell_itr != dt.finite_cells_end(); ++cell_itr) {

		int neighbor;
		if (cell_itr->has_neighbor(dt.infinite_cell(), neighbor)) {
			root_cells.push_back(CellVisit{.cell = cell_itr, .parent = nullptr});
		}

	}


	std::vector<Delaunay::Cell_handle> big_cells;

	for (auto itr = dt.finite_cells_begin(); itr != dt.finite_cells_end(); ++itr) {
		Point circumcenter = itr->circumcenter();
		double sphere_radius_sqr = squared_distance(circumcenter, itr->vertex(0)->point());
		if (sphere_radius_sqr > 0.1 * 0.1) {
			big_cells.push_back(itr);
		}
	}

	Apple a1 = apples[0];
	Apple a2 = apples[42];

	auto c1 = utilities::closest_cell(Point(a1.center.x(), a2.center.y(), a2.center.z()), big_cells);
	auto c2 = utilities::closest_cell(Point(a2.center.x(), a2.center.y(), a2.center.z()), big_cells);

	// Now, perform an A*-


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


