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

	struct Parent {
		Delaunay::Cell_handle cell;
		K::Vector_3 relative;
	};

	using ParentageMap = std::unordered_map<Delaunay::Cell_handle, Parent>;

	ParentageMap generate_parentage(const Delaunay& dt);

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> extract_edges(const ParentageMap& parent_map);
}

#endif //NEW_PLANNERS_DENDRITICCONVEXHULLSHELL_H
