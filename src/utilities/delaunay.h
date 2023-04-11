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
#include <shape_msgs/msg/mesh.hpp>

namespace utilities {

	using K = CGAL::Epick;
	using Triangle = CGAL::Triangle_3<K>;
	using Delaunay3 = CGAL::Delaunay_triangulation_3<K>;

	// Delaunay Triangulation
	CGAL::Delaunay_triangulation_3<K> generateDelaunayTriangulation(const shape_msgs::msg::Mesh &mesh);

	Triangle facet_triangle(const Delaunay3::Cell_handle &cell, int facet_index);

}

#endif //NEW_PLANNERS_DELAUNAY_H
