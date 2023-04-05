// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-3-23.
//

#ifndef NEW_PLANNERS_DENDRITICCONVEXHULLSHELL_H
#define NEW_PLANNERS_DENDRITICCONVEXHULLSHELL_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <queue>

namespace dendritic_convex_hull {

	using K = CGAL::Epick;
	using Delaunay = CGAL::Delaunay_triangulation_3<K>;
	using Point = Delaunay::Point;

	struct CellWithParent {
		Delaunay::Cell_handle parent_cell;
		K::Vector_3 relative;
		double distance_from_chull;
	};

	using ParentageMap = std::unordered_map<Delaunay::Cell_handle, CellWithParent>;

	ParentageMap generate_parentage(const Delaunay& dt);

	struct DendriteNode {
		Eigen::Vector3d position;
		std::vector<std::shared_ptr<DendriteNode>> children;
		std::weak_ptr<DendriteNode> parent;
	};

	std::shared_ptr<DendriteNode> extract_dendrite(const Delaunay::Cell_handle &cell,
												   std::unordered_map<Delaunay::Cell_handle, std::vector<Delaunay::Cell_handle>> &child_map,
												   const std::weak_ptr<DendriteNode> &parent);

	std::vector<std::shared_ptr<DendriteNode>> extract_dendrites(const ParentageMap &parent_map, const Delaunay &dt);

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> extract_dendrite_edges(const DendriteNode &root_node);

	std::shared_ptr<DendriteNode>
	find_closest_node(const std::vector<std::shared_ptr<DendriteNode>> &dendrites, const Eigen::Vector3d &a1);

	std::vector<Eigen::Vector3d> trace_dendrite(std::shared_ptr<DendriteNode> closest_node1);

	CGAL::Surface_mesh<CGAL::Epick::Point_3> extractConvexHullSurfaceMesh(const dendritic_convex_hull::Delaunay &dt);

}

#endif //NEW_PLANNERS_DENDRITICCONVEXHULLSHELL_H
