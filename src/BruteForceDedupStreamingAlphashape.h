

#ifndef NEW_PLANNERS_BRUTEFORCEDEDUPSTREAMINGALPHASHAPE_H
#define NEW_PLANNERS_BRUTEFORCEDEDUPSTREAMINGALPHASHAPE_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>

#include <variant>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>
#include <CGAL/Regular_triangulation_3.h>
#include <shape_msgs/msg/mesh.hpp>
#include "HashedSpatialIndex.h"

class BruteForceDedupStreamingAlphashape {

	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Vb = CGAL::Alpha_shape_vertex_base_3<K>;
	using Fb = CGAL::Alpha_shape_cell_base_3<K>;
	using Tds = CGAL::Triangulation_data_structure_3<Vb, Fb>;
	using Delaunay = CGAL::Delaunay_triangulation_3<K, Tds, CGAL::Fast_location>;
	using Alpha_shape_3 = CGAL::Alpha_shape_3<Delaunay>;
	using Point = K::Point_3;
	using Alpha_iterator = Alpha_shape_3::Alpha_iterator;
	using NT = Alpha_shape_3::NT;

	double threshold;

	HashedSpatialIndex<std::monostate> dedup;

	std::vector<Point> cgal_points;

public:
	explicit BruteForceDedupStreamingAlphashape(double threshold);

private:
	void addPoints(const std::vector<Eigen::Vector3d>& points);

	[[nodiscard]] shape_msgs::msg::Mesh toMesh();

};


#endif //NEW_PLANNERS_BRUTEFORCEDEDUPSTREAMINGALPHASHAPE_H
