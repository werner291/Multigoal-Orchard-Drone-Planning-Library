// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-3-23.
//

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "StarAlphaChullShell.h"
#include "CylinderShell.h"

StarAlphaShell::StarAlphaShell(const AppleTreePlanningScene &scene_info) {

	std::vector<geometry_msgs::msg::Point> points = utilities::extract_leaf_vertices(scene_info);

	std::vector<Point> lwp = points | ranges::views::transform([](const geometry_msgs::msg::Point &p) {
		return Point(p.x, p.y, p.z);
	}) | ranges::to_vector;

	dt.insert(lwp.begin(), lwp.end());

	// Get a list of all terahedra with an empty insphere of a minimum radius.
	const double min_radius = 0.5;

	for (auto itr = dt.cells_begin(); itr != dt.cells_end(); ++itr) {


	}

}

Eigen::Vector3d StarAlphaShell::surface_point(const StarAlphaShellPoint &p) const {

	Point a = p.cell->vertex(0)->point();
	Point b = p.cell->vertex(1)->point();
	Point c = p.cell->vertex(2)->point();
	Point d = p.cell->vertex(3)->point();

	Point bary = CGAL::barycenter(a,
								  p.barycentric_coordinates[0],
								  b,
								  p.barycentric_coordinates[1],
								  c,
								  p.barycentric_coordinates[2],
								  d,
								  p.barycentric_coordinates[3]);

	return {bary.x(), bary.y(), bary.z()};

}

std::shared_ptr<ShellPath<StarAlphaShellPoint>>
StarAlphaShell::path_from_to(const StarAlphaShellPoint &from, const StarAlphaShellPoint &to) const {

	// TODO: Pathfinding only through facets of the tetrahedra.

}

double StarAlphaShell::path_length(const std::shared_ptr<ShellPath<StarAlphaShellPoint>> &path) const {
	return 0;
}

StarAlphaShellPoint StarAlphaShell::nearest_point_on_shell(const Eigen::Vector3d &p) const {

	throw std::runtime_error("Not implemented");

}

Eigen::Vector3d StarAlphaShell::arm_vector(const StarAlphaShellPoint &p) const {
	return (surface_point(p) - global_barycenter).normalized();
}
