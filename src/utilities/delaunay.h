// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-3-23.
//

#ifndef NEW_PLANNERS_DELAUNAY_H
#define NEW_PLANNERS_DELAUNAY_H

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace utilities {

	using Delaunay3 = CGAL::Delaunay_triangulation_3<CGAL::Epick>;

	// Delaunay Triangulation
	CGAL::Delaunay_triangulation_3<CGAL::Epick> generateDelaunayTriangulation(const shape_msgs::msg::Mesh &mesh);

	Delaunay3::Cell_handle closest_cell(const Delaunay3::Point to, std::vector<Delaunay3::Cell_handle> &big_cells);

	std::vector<Delaunay3::Cell_handle> bidirectional_monotonic_astar(Delaunay3::Cell_handle start,
																	  Delaunay3::Cell_handle goal,
																	  std::vector<Delaunay3::Cell_handle> &big_cells);

}

#endif //NEW_PLANNERS_DELAUNAY_H
