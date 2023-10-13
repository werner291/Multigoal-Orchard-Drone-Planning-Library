
#ifndef NEW_PLANNERS_STREAMINGALPHASHAPE_H
#define NEW_PLANNERS_STREAMINGALPHASHAPE_H

#include <variant>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <shape_msgs/msg/mesh.hpp>

class StreamingAlphaShape {

	/// CGAL stuff for the Delaunay triangulation.

	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Vb = CGAL::Alpha_shape_vertex_base_3<K>;
	using Fb = CGAL::Alpha_shape_cell_base_3<K>;
	using Tds = CGAL::Triangulation_data_structure_3<Vb, Fb>;
	using Delaunay = CGAL::Delaunay_triangulation_3<K, Tds, CGAL::Fast_location>;
	using Alpha_shape_3 = CGAL::Alpha_shape_3<Delaunay>;
	using Point = K::Point_3;
	using Alpha_iterator = Alpha_shape_3::Alpha_iterator;
	using NT = Alpha_shape_3::NT;

	Delaunay dt;

	struct UnorderedTriangle {

		Delaunay::Vertex_handle v1;
		Delaunay::Vertex_handle v2;
		Delaunay::Vertex_handle v3;

		UnorderedTriangle(
				Delaunay::Vertex_handle v1,
			Delaunay::Vertex_handle v2,
			Delaunay::Vertex_handle v3) {

			std::array<Delaunay::Vertex_handle, 3> vertices = {v1, v2, v3};
			std::sort(vertices.begin(), vertices.end());

			this->v1 = vertices[0];
			this->v2 = vertices[1];
			this->v3 = vertices[2];

		}

		bool operator==(const UnorderedTriangle &other) const {
			return v1 == other.v1 && v2 == other.v2 && v3 == other.v3;
		}
	};

	struct UnorderedTriangleHash {
		std::size_t operator()(const UnorderedTriangle &triangle) const {
			return std::hash<Delaunay::Vertex_handle>()(triangle.v1) * 31 ^
				(std::hash<Delaunay::Vertex_handle>()(triangle.v2) * 31 * 31) ^
				std::hash<Delaunay::Vertex_handle>()(triangle.v3);
		}
	};

	// Invariant:
	// 	If at least 4 non-coplanar points have been received, then `triangles`
	// 	is the set of all triangles on the surface of the alpha shape.
	std::unordered_set<UnorderedTriangle, UnorderedTriangleHash> triangles;

public:
	void addPoint(const Eigen::Vector3d &points);

	/**
	 * \brief Return a ROS mesh message of the outer boundary of the Alpha shape.
	 */
	shape_msgs::msg::Mesh extractSurfaceMesh();

	struct Inside;

	struct OnSurface {

	};

	StreamingAlphaShape(double margin);
};


#endif //NEW_PLANNERS_STREAMINGALPHASHAPE_H
