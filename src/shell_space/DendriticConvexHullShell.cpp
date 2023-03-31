
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
	std::unordered_map<Delaunay::Cell_handle, CellWithParent> parent_map;

	struct PotentialParentage {
		Delaunay::Cell_handle proposed_parent, proposed_child;
		K::Vector_3 relative;
		double distance_from_chull = 0.0;
	};

	std::priority_queue<PotentialParentage, std::vector<PotentialParentage>, std::function<bool(const PotentialParentage&, const PotentialParentage&)>> potential_parentage_queue([](const PotentialParentage &a, const PotentialParentage &b) {
		return a.distance_from_chull > b.distance_from_chull;
	});

	for (auto itr = dt.finite_cells_begin(); itr != dt.finite_cells_end(); ++itr) {
		for (int i = 0; i < 4; ++i) {
			if (dt.is_infinite(itr->neighbor(i))) {

				// Get the barycenter of the current one.
				Point barycenter = CGAL::centroid(
						itr->vertex(0)->point(),
						itr->vertex(1)->point(),
						itr->vertex(2)->point(),
						itr->vertex(3)->point()
						);

				// Compute the distance to the hull, which we shall take to mean the distance to the barycenter of
				// of the facet that is adjacent to the infinite cell, aka, the i-th facet thanks to the way we're
				// iterating over the cells.

				// Get the barycenter of the facet.
				Point facet_barycenter = CGAL::centroid(
						itr->vertex((i + 1) % 4)->point(),
						itr->vertex((i + 2) % 4)->point(),
						itr->vertex((i + 3) % 4)->point()
						);

				// Compute the distance.
				double distance_to_hull = CGAL::sqrt(CGAL::squared_distance(barycenter, facet_barycenter));

				Delaunay::Cell_handle parent_cell = dt.infinite_cell();
				Delaunay::Cell_handle child_cell = itr;

				potential_parentage_queue.push({
					parent_cell,
					child_cell,
					barycenter - facet_barycenter,
					distance_to_hull
				});
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
			if (parent_map[potential_parentage.proposed_child].distance_from_chull <= potential_parentage.distance_from_chull) {
				continue;
			}
		}

		// Otherwise, add it to the parent map.
		parent_map[potential_parentage.proposed_child] = {
				potential_parentage.proposed_parent,
				potential_parentage.relative,
				potential_parentage.distance_from_chull
		};

		// Now, add all of the child's neighbors to the queue.
		for (int i = 0; i < 4; ++i) {
			Delaunay::Cell_handle neighbor = potential_parentage.proposed_child->neighbor(i);

			K::Point_3 own_barycenter = CGAL::centroid(
					potential_parentage.proposed_child->vertex(0)->point(),
					potential_parentage.proposed_child->vertex(1)->point(),
					potential_parentage.proposed_child->vertex(2)->point(),
					potential_parentage.proposed_child->vertex(3)->point()
					);

			if (neighbor == dt.infinite_cell()) {
				continue;
			}

			K::Point_3 neighbor_barycenter = CGAL::centroid(neighbor->vertex(0)->point(), neighbor->vertex(1)->point(), neighbor->vertex(2)->point(), neighbor->vertex(3)->point());

			K::Vector_3 relative = own_barycenter - neighbor_barycenter;

			K::Point_3 facet_circumcenter = CGAL::circumcenter(potential_parentage.proposed_child->vertex((i + 1) % 4)->point(), potential_parentage.proposed_child->vertex((i + 2) % 4)->point(), potential_parentage.proposed_child->vertex((i + 3) % 4)->point());
			double facet_circumradius = CGAL::sqrt(CGAL::squared_distance(facet_circumcenter, potential_parentage.proposed_child->vertex((i + 1) % 4)->point()));

			double transition_cost = CGAL::sqrt(relative.squared_length());// / facet_circumradius;

			// Otherwise, add it to the queue.
			potential_parentage_queue.push({
				potential_parentage.proposed_child,
				neighbor,
				relative,
				potential_parentage.distance_from_chull + transition_cost
			});
		}

	}

	return parent_map;
}

std::shared_ptr<dendritic_convex_hull::DendriteNode> dendritic_convex_hull::extract_dendrite(const Delaunay::Cell_handle &cell,
																			std::unordered_map<Delaunay::Cell_handle, std::vector<Delaunay::Cell_handle>> &child_map,
																							 const std::weak_ptr<dendritic_convex_hull::DendriteNode> &parent) {

	Delaunay::Point barycenter = CGAL::centroid(cell->vertex(0)->point(), cell->vertex(1)->point(), cell->vertex(2)->point(), cell->vertex(3)->point());

	auto node = std::make_shared<dendritic_convex_hull::DendriteNode>();

	node->position = {barycenter.x(), barycenter.y(), barycenter.z()};
	for (const auto &child: child_map[cell]) {
		node->children.push_back(extract_dendrite(child, child_map, node));
	}
	return node;
}

std::vector<std::shared_ptr<dendritic_convex_hull::DendriteNode>>
dendritic_convex_hull::extract_dendrites(const dendritic_convex_hull::ParentageMap &parent_map,
										 const dendritic_convex_hull::Delaunay &dt) {

	std::unordered_map<Delaunay::Cell_handle, std::vector<Delaunay::Cell_handle>> child_map;

	for (const auto & itr : parent_map) {
		child_map[itr.second.parent_cell].push_back(itr.first);
	}

	std::vector<std::shared_ptr<DendriteNode>> dendrites;

	for (const auto &itr : parent_map) {
		if (itr.second.parent_cell == dt.infinite_cell()) {
			dendrites.push_back(extract_dendrite(itr.first, child_map,std::weak_ptr<DendriteNode>()));
		}
	}

	return dendrites;

}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
dendritic_convex_hull::extract_dendrite_edges(const dendritic_convex_hull::DendriteNode &root_node) {

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

	for (const auto& child : root_node.children) {
		edges.emplace_back(root_node.position, child->position);
		auto child_edges = extract_dendrite_edges(*child);
		edges.insert(edges.end(), child_edges.begin(), child_edges.end());
	}

	return edges;

}
