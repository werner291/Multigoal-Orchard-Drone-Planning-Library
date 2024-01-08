// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/7/24.
//

#include "geometry.h"
#include "spherical_geometry.h"
#include <algorithm>

namespace mgodpl::spherical_geometry {

	double latitude(const Edge &edge, double longitude)  {
		// Just like intersection_longitude, we can reduce this problem to linear algebra.

		// Compute the edge plane normal.
		math::Vec3d edge_normal = edge.vertices[0].cross(edge.vertices[1]);

		math::Vec3d lon_direction = math::Vec3d(
				cos(longitude + M_PI / 2.0),
				sin(longitude + M_PI / 2.0),
				0
		);

		// Now it's easy: get the direction of the intersection line.
		math::Vec3d direction = edge_normal.cross(lon_direction);

		// If the dot product with the edge vertices is negative, flip the direction.
		if (direction.dot(edge.vertices[0]) < 0) {
			assert(direction.dot(edge.vertices[1]) < 0);
			direction = -direction;
		}

		return latitude(direction, math::Vec3d(0, 0, 0));
	}

	std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle &triangle, const math::Vec3d &center) {

		std::array<RelativeVertex, 3> vertices{
				RelativeVertex {
						longitude(triangle.vertices[0], center), latitude(triangle.vertices[0], center),
				},
				RelativeVertex{
						longitude(triangle.vertices[1], center), latitude(triangle.vertices[1], center),
				},
				RelativeVertex{
						longitude(triangle.vertices[2], center), latitude(triangle.vertices[2], center),
				}
		};

		assert(-M_PI/2.0 <= vertices[0].latitude && vertices[0].latitude <= M_PI/2.0);
		assert(-M_PI/2.0 <= vertices[1].latitude && vertices[1].latitude <= M_PI/2.0);
		assert(-M_PI/2.0 <= vertices[2].latitude && vertices[2].latitude <= M_PI/2.0);

		std::sort(vertices.begin(), vertices.end(), [](const auto &a, const auto &b) {
			return signed_longitude_difference(a.longitude, b.longitude) < 0;
		});

		return vertices;
	}

	PaddedSphereTriangle
	PaddedSphereTriangle::from_triangle(const Triangle &triangle, const math::Vec3d &center, double arm_radius) {

		auto sorted = sorted_relative_vertices(triangle, center);

		double triangle_distance_from_center =
				std::min((triangle.vertices[0] - center).norm(),
						 std::min((triangle.vertices[1] - center).norm(),
								  (triangle.vertices[2] - center).norm()));

		double padding = mgodpl::spherical_geometry::angular_padding(arm_radius, triangle_distance_from_center);

		return {
				{
						RelativeVertex{sorted[0].longitude, sorted[0].latitude},
						RelativeVertex{sorted[1].longitude, sorted[1].latitude},
						RelativeVertex{sorted[2].longitude, sorted[2].latitude}
				},
				padding
		};

	}

	LatitudeRange PaddedSphereTriangle::occupied_latitudes(const LongitudeRange &lon) {
		throw std::runtime_error("Not implemented");
//		double l1 = this->vertices[0].longitude;
//		double l2 = this->vertices[1].longitude;
//		double l3 = this->vertices[2].longitude;
//		double l0 = wrap_angle(l1 - angular_padding);
//		double l4 = wrap_angle(l3 + angular_padding);
//
//		assert(lon.overlaps(LongitudeRange(l0, l4)));

//			auto long_restricted = edge_long.restrict(longitude_range_of_cell);
//
//			spherical_geometry::LatitudeRange latitude_range_of_cell = long_restricted.latitude_range();
//
//			if (longitude_range_of_cell.contains(triangle.vertices[1].longitude)) {
//				latitude_range_of_cell.min = std::min(latitude_range_of_cell.min, triangle.vertices[1].latitude);
//				latitude_range_of_cell.max = std::max(latitude_range_of_cell.max, triangle.vertices[1].latitude);
//			} else {
//				if (spherical_geometry::signed_longitude_difference(triangle.vertices[1].longitude,
//																	longitude_range_of_cell.start) < 0) {
//					latitude_range_of_cell.min = std::min(latitude_range_of_cell.min, triangle.vertices[1].latitude);
//				} else {
//					latitude_range_of_cell.max = std::max(latitude_range_of_cell.max, triangle.vertices[1].latitude);
//				}
//			}
	}
}
