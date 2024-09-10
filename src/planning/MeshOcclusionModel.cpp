//
// Created by werner on 27-2-23.
//

#include <random>
#include "MeshOcclusionModel.h"

namespace mgodpl {

	MeshOcclusionModel::MeshOcclusionModel(const Mesh &mesh, double margin) : margin(margin) {

		// Use CGAL to create an AABB tree of the mesh's triangles.

		for (const auto &triangle: mesh.triangles) {
			const auto &p1 = mesh.vertices[triangle[0]];
			const auto &p2 = mesh.vertices[triangle[1]];
			const auto &p3 = mesh.vertices[triangle[2]];

			// Skip degenerate triangles:
			Point a(p1.x(), p1.y(), p1.z());
			Point b(p2.x(), p2.y(), p2.z());
			Point c(p3.x(), p3.y(), p3.z());

			Triangle t(a, b, c);

			if (!t.is_degenerate()) {
				triangles.push_back(t);
			}
		}

		tree.insert(triangles.begin(), triangles.end());

	}

	bool MeshOcclusionModel::checkOcclusion(const math::Vec3d &point, const math::Vec3d &viewpoint) const {

		// Actually check a point that's `margin` away from the point, in the direction of the viewpoint.
		// This is to prevent the point from being occluded by accidentally being inside of the mesh.

		// Create a segment from the viewpoint to the point.
		const auto direction = (point - viewpoint).normalized();
		const auto offset = direction * margin;
		const auto offsetPoint = point - offset;

		Segment segment(Point(viewpoint.x(), viewpoint.y(), viewpoint.z()),
						Point(offsetPoint.x(), offsetPoint.y(), offsetPoint.z()));

		// Check if the line intersects the mesh.
		return tree.do_intersect(segment);

	}

	double MeshOcclusionModel::exteriorVisibilityScore(const math::Vec3d &apple, const int n_samples) {
		// Create a random number generator and a normal distribution
		std::random_device rd;
		std::mt19937 gen(rd());
		std::normal_distribution<> d(0, 1);

		// Initialize a counter for the number of unoccluded lines
		double visibility_factor = 0;

		// Loop through 1000 randomly generated directions and test for occlusion
		for (int i = 0; i < n_samples; i++) {
			// Generate a random direction vector and normalize it
			math::Vec3d direction(d(gen), d(gen), d(gen));
			direction.normalize();

			// Scale the direction vector to a length of 10
			direction *= 10.0;

			// Test for occlusion between the apple and a point along the direction vector
			bool occluded = checkOcclusion(apple, apple + direction);

			// Increment the counter if the line is unoccluded
			visibility_factor += occluded ? 0 : 1;
		}

		// Normalize the visibility factor by the number of tests and return it
		visibility_factor /= 1000.0;
		return visibility_factor;
	}

}
