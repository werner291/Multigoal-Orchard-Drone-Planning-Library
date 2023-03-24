
#include "DendriticConvexHullShell.h"

#include <range/v3/all.hpp>

dendritic_convex_hull::ParentageMap dendritic_convex_hull::generate_parentage(const dendritic_convex_hull::Delaunay &dt) {

	// Now, we'll want to compute a tree of the shortest barycenter-to-barycenter paths, rooted on
	// the surface of the convex hull, to the barycenter of each cell. We'll use a Dijkstra-like
	// algorithm to do this.

	// We'll start by finding all cells that are on the convex hull. Technically, these are the cells adjacent
	// to the infinite cell, but CGAL doesn't provide a way to get these cells directly (the infinite cell apparently
	// only has max 4 neighbors, which makes no sense.) Instead, we'll just iterate over all cells and check if one
	// of its neighbors is the infinite cell.
	std::unordered_map<Delaunay::Cell_handle, Parent> parent_map;

	struct PotentialParentage {
		Delaunay::Cell_handle proposed_parent, proposed_child;
		K::Vector_3 relative;
	};

	std::priority_queue<PotentialParentage, std::vector<PotentialParentage>, std::function<bool(const PotentialParentage&, const PotentialParentage&)>> potential_parentage_queue([](const PotentialParentage &a, const PotentialParentage &b) {
		return a.relative.squared_length() > b.relative.squared_length();
	});

	for (auto itr = dt.finite_cells_begin(); itr != dt.finite_cells_end(); ++itr) {
		for (int i = 0; i < 4; ++i) {
			if (itr->neighbor(i) == dt.infinite_cell()) {

				// Get the barycenter of the current one.
				Point barycenter = CGAL::centroid(itr->vertex(0)->point(), itr->vertex(1)->point(), itr->vertex(2)->point(), itr->vertex(3)->point());

				// Compute the distance to the hull, which we shall take to mean the distance to the barycenter of
				// of the facet that is adjacent to the infinite cell, aka, the i-th facet thanks to the way we're
				// iterating over the cells.

				// Get the barycenter of the facet.
				Point facet_barycenter = CGAL::centroid(itr->vertex((i + 1) % 4)->point(), itr->vertex((i + 2) % 4)->point(), itr->vertex((i + 3) % 4)->point());

				// Compute the distance.
				double distance_to_hull = CGAL::sqrt(CGAL::squared_distance(barycenter, facet_barycenter));

				Delaunay::Cell_handle parent_cell = dt.infinite_cell();
				Delaunay::Cell_handle child_cell = itr;

				potential_parentage_queue.push({parent_cell, child_cell, facet_barycenter - barycenter});
				break;
			}
		}
	}

	while (!potential_parentage_queue.empty()) {

		PotentialParentage potential_parentage = potential_parentage_queue.top();
		potential_parentage_queue.pop();

		// Check if we've already processed this cell.
		if (parent_map.find(potential_parentage.proposed_child) != parent_map.end()) {
			// Check if our distance is better. If not, we can just skip this one.
			if (parent_map[potential_parentage.proposed_child].relative.squared_length() <= potential_parentage.relative.squared_length()) {
				continue;
			}
		}

		// Otherwise, add it to the parent map.
		parent_map[potential_parentage.proposed_child] = {potential_parentage.proposed_parent, potential_parentage.relative};

		// Now, add all of the child's neighbors to the queue.
		for (int i = 0; i < 4; ++i) {
			Delaunay::Cell_handle neighbor = potential_parentage.proposed_child->neighbor(i);

			K::Point_3 neighbor_barycenter = CGAL::centroid(neighbor->vertex(0)->point(), neighbor->vertex(1)->point(), neighbor->vertex(2)->point(), neighbor->vertex(3)->point());

			// Otherwise, add it to the queue.
			potential_parentage_queue.push({potential_parentage.proposed_child, neighbor, neighbor_barycenter - potential_parentage.proposed_child->vertex(i)->point()});
		}

	}

	return parent_map;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
dendritic_convex_hull::extract_edges(const dendritic_convex_hull::ParentageMap &parent_map) {

	// Now, extract the parentage relationships. For now, we'll just collect the barycenter-to-barycenter edges in a vector of Eigen::Vector3d pairs.
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> barycenter_to_barycenter_edges;

	for (const auto & itr : parent_map) {
		// Get the barycenter of the current one.
		Point barycenter = CGAL::centroid(itr.first->vertex(0)->point(),
										  itr.first->vertex(1)->point(),
										  itr.first->vertex(2)->point(),
										  itr.first->vertex(3)->point());

		// Get the barycenter of the parent, relative (since the ones on the boundary with the infinite cell are a bit screwy).

		Point parent_barycenter = barycenter + itr.second.relative;

		barycenter_to_barycenter_edges.emplace_back(
				Eigen::Vector3d(barycenter.x(), barycenter.y(), barycenter.z()),
				Eigen::Vector3d(parent_barycenter.x(),
								parent_barycenter.y(),
								parent_barycenter.z()));

	}

	return barycenter_to_barycenter_edges;

}
