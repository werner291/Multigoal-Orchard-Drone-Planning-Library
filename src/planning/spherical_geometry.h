// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/7/24.
//

#ifndef MGODPL_SPHERICAL_GEOMETRY_H
#define MGODPL_SPHERICAL_GEOMETRY_H

#include <array>
#include <algorithm>
#include "geometry.h"

//
//math::Vec3d Triangle::normal() const {
//	return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
//}
//
//inline double latitude(const math::Vec3d &point, const math::Vec3d &center) {
//	math::Vec3d delta = point - center;
//
//	// Distance from the vertical axis through the center of the sphere.
//	const double distance_xy = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
//
//	// Compute the latitude.
//	return std::atan2(delta.z(), distance_xy);
//}
//
//inline double longitude(const math::Vec3d &point, const math::Vec3d &center) {
//	math::Vec3d delta = point - center;
//
//	return std::atan2(delta.y(), delta.x());
//}
//
//double angular_padding(double arm_radius, double obstacle_distance) {
//	return atan(arm_radius / obstacle_distance);
//}
//
//inline double latitude(const Edge &edge, double at_longitude) {
//	// Just like intersection_longitude, we can reduce this problem to linear algebra.
//
//	// Compute the edge plane normal.
//	math::Vec3d edge_normal = edge.vertices[0].cross(edge.vertices[1]);
//
//	math::Vec3d lon_direction = math::Vec3d(
//			cos(at_longitude + M_PI / 2.0),
//			sin(at_longitude + M_PI / 2.0),
//			0
//	);
//
//	// Now it's easy: get the direction of the intersection line.
//	math::Vec3d direction = edge_normal.cross(lon_direction);
//
//	// If the dot product with the edge vertices is negative, flip the direction.
//	if (direction.dot(edge.vertices[0]) < 0) {
//		assert(direction.dot(edge.vertices[1]) < 0);
//		direction = -direction;
//	}
//
//	return latitude(direction, math::Vec3d(0, 0, 0));
//}
//
//inline double interpolate_longitude(double first, double second, double t) {
//	double diff = signed_longitude_difference(second, first);
//	assert(diff >= 0);
//	return wrap_angle(first + t * diff);
//}
//
//inline double wrap_angle(double angle) {
//	assert(angle >= -3 * M_PI && angle <= 3 * M_PI);
//	if (angle > M_PI) {
//		return angle - 2 * M_PI;
//	} else if (angle < -M_PI) {
//		return angle + 2 * M_PI;
//	} else {
//		return angle;
//	}
//}

namespace mgodpl::spherical_geometry {

	struct Latitude {
		double latitude;
		Latitude(double latitude) : latitude(latitude) {
			assert(latitude >= -M_PI / 2.0 && latitude <= M_PI / 2.0);
		}
	};

	struct Longitude {
		double longitude;
		Longitude(double longitude) : longitude(longitude) {
			assert(longitude >= -M_PI && longitude <= M_PI);
		}
	};

	/**
	 * \brief A method that computes the angular padding to add to a given polar obstacle point.
	 *
	 * To think about this conceptually, imagine a cylinder of radius r, and a polar point (lat, lon, r).
	 *
 	 * Suppose that the center of one of th bases of the cylinder is at the origin; what are the lat/lon of the median
	 * line of the cylinder, assuming that the polar obstacle point is on the surface of the cylinder?
	 *
	 * Effectively, we have a right-angled triangle with the hypotenuse r, and the other leg of length arm_radius;
	 * we're looking for the angle between the hypotenuse and the leg of length r.
	 *
	 * That's just atan(arm_radius / r).
	 */
	inline double angular_padding(double arm_radius, double obstacle_distance) {
		return atan(arm_radius / obstacle_distance);
	}

	/**
	 * \brief Given an angle, wrap it into the range [-pi, pi].
	 *
	 * Precodition: the given angle must be in the range [-3pi, 3pi].
	 *
	 * @param angle 		The angle to wrap.
	 * @return 				The wrapped angle.
	 */
	inline double wrap_angle(double angle) {
		assert(angle >= -3 * M_PI && angle <= 3 * M_PI);
		if (angle > M_PI) {
			return angle - 2 * M_PI;
		} else if (angle < -M_PI) {
			return angle + 2 * M_PI;
		} else {
			return angle;
		}
	}

	/**
	 * @brief   Compute the latitude of the given point, if projected onto a sphere centered at the given center.
	 *
	 * @param   point   The point to compute the latitude of.
	 * @param   center  The center of the sphere.
	 *
	 * @return  The latitude, as a double in the range [-pi/2, pi/2].
	 */
	inline double latitude(const math::Vec3d &point, const math::Vec3d &center = {0,0,0}) {
		math::Vec3d delta = point - center;

		// Distance from the vertical axis through the center of the sphere.
		const double distance_xy = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

		// Compute the latitude.
		return std::atan2(delta.z(), distance_xy);
	}

	/**
	 * \brief Compute the longitude of the projection of a point on a sphere (poles on the Z-axis).
 	 *
 	 * \param point     The point to compute the longitude of.
	 * \param center    The center of the sphere.
	 *
	 * \return          The longitude, as a double in the range [-pi, pi].
	 */
	inline double longitude(const math::Vec3d &point, const math::Vec3d &center = {0,0,0}) {
		math::Vec3d delta = point - center;

		return std::atan2(delta.y(), delta.x());
	}

//	/**
//	* \brief Given an edge, return the latitude of the intersection of the projection of the edge on the sphere with the longitude sweep arc at the given longitude.
//	* \param range		    The range of longitudes over which the latitude is valid.
//	* \param longitude	    The longitude at which to compute the latitude.
//	* \return		        The latitude of the intersection of the projection of the edge on the sphere with the given longitude.
//	*/
//	double latitude(const mgodpl::Edge &edge, double longitude);

	/**
	 * \brief Compute the signed difference between two longitudes.
	 *
	 * That is: compute the difference between the two longitudes, and put it into the range [-pi, pi].
	 *
	 * \return The signed difference between the two longitudes, as a double in the range [-pi, pi].
	 */
	inline double signed_longitude_difference(double first, double second) {
		// Compute the difference:
		double difference = first - second;

		// Put it into the range [-pi, pi]
		if (difference > M_PI) {
			difference -= 2 * M_PI;
		} else if (difference < -M_PI) {
			difference += 2 * M_PI;
		}

		return difference;
	}


	/**
	 * \brief  Compute the longitude of the given point, relative to the given starting longitude.
	 * \param starting_longitude  The starting longitude of the sweep.
	 * \param longitude           The longitude to compute the relative longitude of.
	 * \return                    The relative longitude, as a double in the range [0, 2pi].
	 */
	inline double longitude_ahead_angle(const double starting_longitude, const double longitude) {
		double a_longitude = signed_longitude_difference(longitude, starting_longitude);
		if (a_longitude < 0) a_longitude += 2 * M_PI;
		return a_longitude;
	}

	/**
	* \brief A struct representing a vertex in a triangle, relative to some center, with the longitude of the vertex.
	*/
	struct RelativeVertex {
		double longitude = 0.0;
		double latitude = 0.0;

		// Stream output operator:
		friend std::ostream& operator<<(std::ostream& os, const RelativeVertex& vertex) {
			os << "RelativeVertex(longitude=" << vertex.longitude << ", latitude=" << vertex.latitude << ")";
			return os;
		}

		[[nodiscard]] math::Vec3d to_cartesian() const {
			return {
					cos(latitude) * cos(longitude),
					cos(latitude) * sin(longitude),
					sin(latitude)
			};
		}
	};

	/**
	* \brief Compute the three relative vertices of a triangle, sorted.
	*/
	std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle &triangle,
														   const math::Vec3d &center);


	/**
	 * @brief A range of latitudes between a lower and an upper bound.
	 */
	struct LatitudeRange {

		/// The lower and upper bounds of the latitude range.
		/// Both are in the range [-pi, pi], and min <= max.
		double min, max;

		LatitudeRange(double min, double max) : min(min), max(max) {
			assert(min <= max);
		}

		[[nodiscard]] inline bool contains(double latitude) const {
			return latitude >= min && latitude <= max;
		}

		[[nodiscard]] inline double interpolate(double t) const {
			assert(t >= 0 && t <= 1);
			return min + t * (max - min);
		}

		[[nodiscard]] inline bool overlaps(const LatitudeRange &other) const {
			return contains(other.min) || contains(other.max) || other.contains(min) || other.contains(max);
		}

		const double clamp(double d) {
			return std::clamp(d, min, max);
		}
	};

	struct LongitudeRange {
		double start, end;

		LongitudeRange(double start, double end) : start(start), end(end) {
			assert(-M_PI <= start && start <= M_PI);
			assert(-M_PI <= end && end <= M_PI);
		}

		[[nodiscard]] bool contains(double longitude) const {
			assert(-M_PI <= longitude && longitude <= M_PI);
			if (start <= end) {
				return longitude >= start && longitude <= end;
			} else {
				return longitude >= start || longitude <= end;
			}
		}

		[[nodiscard]] bool contains(const Longitude& longitude) const {
			return contains(longitude.longitude);
		}

		[[nodiscard]] bool overlaps(const LongitudeRange &other) const {
			return contains(other.start) || contains(other.end) || other.contains(start) || other.contains(end);
		}

		[[nodiscard]] double clamp(double a) const {

			if (contains(a)) {
				return a;
			} else {
				double compl_t = complement().reverse_interpolate(a);
				assert(compl_t >= 0 && compl_t <= 1);

				if (compl_t > 0.5) {
					return start;
				} else {
					return end;
				}
			}

		}

		[[nodiscard]] LongitudeRange complement() const {
			LongitudeRange to_return = *this;
			std::swap(to_return.start, to_return.end);
			return to_return;
		}

		[[nodiscard]] LongitudeRange restrict(const LongitudeRange &other) const {
			return LongitudeRange(std::max(start, other.start), std::min(end, other.end));
		}

		[[nodiscard]] LongitudeRange overlap(const LongitudeRange &other) const {
			return LongitudeRange(signed_longitude_difference(start, other.start) < 0 ? other.start : start,
								  signed_longitude_difference(end, other.end) > 0 ? other.end : end);
		}

		// Stream output operator:
		friend std::ostream& operator<<(std::ostream& os, const LongitudeRange& range) {
			os << "LongitudeRange(start=" << range.start << ", end=" << range.end << ")";
			return os;
		}

		[[nodiscard]] double length() const {
			if (start <= end) {
				return end - start;
			} else {
				return 2 * M_PI - start + end;
			}
		}

		[[nodiscard]] Longitude interpolate(double t) const {
			assert(t >= 0 && t <= 1);
			if (t == 0) {
				return start;
			} else if (t == 1) { // Handling these as a special case as numerical errors can cause problems here.
				return end;
			} else {
				return wrap_angle(start + t * length());
			}
		}

		[[nodiscard]] double reverse_interpolate(const double d) const {
			assert(contains(d));

			if (start <= end) {
				return (d - start) / length();
			} else {
				if (d >= start) {
					return (d - start) / length();
				} else {
					return (d + 2 * M_PI - start) / length();
				}
			}
		}
	};

	static int next_id = 0;

	/**
	 * \brief A struct representing an arc edge across the surface of a sphere, defined by its start and end vertices.
	 *
	 * The start and end vertices are defined by a longitude and a latitude, and are sorted by longitude.
	 */
	struct OrderedArcEdge {

		int id;
		RelativeVertex start, end;
		math::Vec3d start_cartesian, end_cartesian;
		double angle;

		OrderedArcEdge(const RelativeVertex &start, const RelativeVertex &end) :
			id(next_id++),
			start(start),
			end(end),
			start_cartesian(start.to_cartesian()),
			end_cartesian(end.to_cartesian()),
			angle(acos(start_cartesian.dot(end_cartesian))) {
		}

		[[nodiscard]] double latitudeAtLongitude(double longitude) const {
			// TODO: This is a naive linear interpolation; this should work for now but isn't quite correct.
			assert(longitude_range().contains(longitude));

			double t = longitude_range().reverse_interpolate(longitude);

			// Apply slerp correction:
			double t1 = sin((1.0-t)*angle)/sin(angle);
			double t2 = sin(t*angle)/sin(angle);

			return start.latitude * t1 + end.latitude * t2;
		}

		[[nodiscard]] LongitudeRange longitude_range() const {
			return {start.longitude, end.longitude};
		}

		[[nodiscard]] LatitudeRange latitude_range() const {
			return {std::min(start.latitude, end.latitude), std::max(start.latitude, end.latitude)};
		}

		/**
		 * \brief Check whether this edge crosses the given edge.
		 *
		 * That is, the latitude of `other` is below the latitude of this edge at the end of their shared longitude range.
		 *
		 * @pre 	The longitude ranges of the two edges_padded overlap.
		 * @pre     The latitude at `after_longitude` is below `other`.
		 * @pre     The given `after_longitude` is in the longitude range of both edges_padded.
		 *
		 * @param other			  The edge to check for crossing.
		 * @param after_longitude The longitude after which to check for crossing.
		 * @return 		          Whether the edges_padded cross.
		 */
		[[nodiscard]] bool crosses(const OrderedArcEdge &other, double after_longitude) const {
			assert(longitude_range().overlaps(other.longitude_range()));
			assert(longitude_range().contains(after_longitude));

			auto overlap = longitude_range().overlap(other.longitude_range());

			assert(latitudeAtLongitude(after_longitude) <= other.latitudeAtLongitude(after_longitude));

			double l1_end = latitudeAtLongitude(overlap.end);
			double l2_end = other.latitudeAtLongitude(overlap.end);
			return l1_end > l2_end;
		}

		/**
		 * \brief Compute the intersection of this edge with the given edge, assuming that they do cross.
		 *
		 * @pre The edges_padded cross.
		 *
		 * @param other	The edge to compute the intersection with.
		 * @return	    The intersection of the two edges_padded.
		 */
		[[nodiscard]] RelativeVertex intersection(const OrderedArcEdge &other) const {

			// Check the precondition that the edges cross:
			assert(crosses(other, this->longitude_range().overlap(other.longitude_range()).start));

			// Extract the longitudes and latitudes of the edges:
			double lat1end = end.latitude;
			double lat1start = start.latitude;
			double lat2end = other.end.latitude;
			double lat2start = other.start.latitude;

			double lon1end = end.longitude;
			double lon1start = start.longitude;
			double lon2end = other.end.longitude;
			double lon2start = other.start.longitude;

			// If the edges cross the pi/-pi boundary, we need to adjust the longitudes:
			if (lon1end < lon1start) {
				if (lon1start < lon2end) {
					lon1end += 2 * M_PI;
				} else {
					lon1start -= 2 * M_PI;
				}
			}

			// If the edges cross the pi/-pi boundary, we need to adjust the longitudes:
			if (lon2end < lon2start) {
				if (lon2start < lon1end) {
					lon2end += 2 * M_PI;
				} else {
					lon2start -= 2 * M_PI;
				}
			}

			// Compute the slopes and intercepts of the lines:
			double m1 = (lat1end - lat1start) / (lon1end - lon1start);
			double p1 = lat1start - m1 * lon1start;

			double m2 = (lat2end - lat2start) / (lon2end - lon2start);
			double p2 = lat2start - m2 * lon2start;

			// Compute the intersection: (TODO: this is a naive linear interpolation; this should work for now but isn't quite correct.)
			double longitude = (p2 - p1) / (m1 - m2);
			double latitude = m1 * longitude + p1;

			// Wrap the longitude into the range [-pi, pi] and return the intersection:
			return {wrap_angle(longitude), latitude};
		}

		/**
		 * \brief Check whether the given point is on this arc edge.
		 *
		 * @pre The point falls in the longitude range of this edge.
		 *
		 * @param v 			The point to check.
		 * @param margin 		The margin to use for the check.
		 * @return 				Whether the point is on the edge.
		 */
		[[nodiscard]] bool is_on_edge(const RelativeVertex v, const double margin = 1e-6) const {
			double t = LongitudeRange(start.longitude, end.longitude).reverse_interpolate(v.longitude);
			double lat = start.latitude * (1 - t) + end.latitude * t;
			return std::abs(lat - v.latitude) < margin;
		}

		// Stream output operator:
		friend std::ostream& operator<<(std::ostream& os, const OrderedArcEdge& edge) {
			os << "OrderedArcEdge(id = " << edge.id << ", start=" << edge.start << ", end=" << edge.end << ")";
			return os;
		}

		/**
		 * Restrict the given edge such that it starts and ends within the given longitude range.
		 *
		 * Precondition: at least some part of the edge must lie within the given longitude range.
		 *
		 * If it lies completely within, it is returned as-is.
		 *
		 * If it lies partially within, the portion that lies outside is trimmed.
		 */
		[[nodiscard]] OrderedArcEdge restrict(const LongitudeRange &range) const {

			if (range.contains(start.longitude) && range.contains(end.longitude)) {
				return *this;
			}

			if (range.contains(start.longitude)) {
				return {start, {range.end, latitudeAtLongitude(range.end)}};
			}

			if (range.contains(end.longitude)) {
				return {{range.start, latitudeAtLongitude(range.start)}, end};
			}

			return {{
				range.start,
				latitudeAtLongitude(range.start)
			}, {
				range.end,
				latitudeAtLongitude(range.end)
			}};

		}

		/**
		 * Restrict the given edge to the given latitude range.
		 *
		 * Precondition: at least some part of the edge must lie within the given latitude range.
		 */
		[[nodiscard]] OrderedArcEdge restrict(const LatitudeRange &range) const {
			throw std::runtime_error("Not implemented");
		}
	};

	/// A spherical triangle with an angular padding around it.
	struct PaddedSphereTriangle {
		/// The three vertices of the triangle.
		std::array<RelativeVertex, 3> vertices;
		// The padding, in radians.
		double angular_padding;

		PaddedSphereTriangle(const std::array<RelativeVertex, 3> &vertices, double angular_padding)
				: vertices(vertices), angular_padding(angular_padding) {}

		static PaddedSphereTriangle from_triangle(const Triangle &triangle,
												  const mgodpl::math::Vec3d &center,
												  double arm_radius);

		[[nodiscard]] LatitudeRange latitude_range_over_longitude_range(const LongitudeRange &range) const {
			double min_lat = std::numeric_limits<double>::max();
			double max_lat = std::numeric_limits<double>::lowest();

			auto edges = edges_padded();

			assert(edges.e_long.longitude_range().overlaps(range));

			auto lats_1 = edges.e_long.restrict(range);
			min_lat = std::min(min_lat, lats_1.latitude_range().min);
			max_lat = std::max(max_lat, lats_1.latitude_range().max);

			if (edges.e_short1.longitude_range().overlaps(range)) {
				auto lats_2 = edges.e_short1.restrict(range);
				min_lat = std::min(min_lat, lats_2.latitude_range().min);
				max_lat = std::max(max_lat, lats_2.latitude_range().max);
			}

			if (edges.e_short2.longitude_range().overlaps(range)) {
				auto lats_3 = edges.e_short2.restrict(range);
				min_lat = std::min(min_lat, lats_3.latitude_range().min);
				max_lat = std::max(max_lat, lats_3.latitude_range().max);
			}

			// Make sure we actually found a range:
			assert(min_lat <= max_lat);

			return {min_lat, max_lat};
		}

		[[nodiscard]] std::pair<LatitudeRange, LongitudeRange> bounding_rectangle() const {
			return { latitude_range(), longitude_range() };
		}

		[[nodiscard]] spherical_geometry::LatitudeRange latitude_range() const {
			return {
					std::min(std::min(vertices[0].latitude, vertices[1].latitude), vertices[2].latitude) -
					angular_padding,
					std::max(std::max(vertices[0].latitude, vertices[1].latitude), vertices[2].latitude) +
					angular_padding
							};
		}

		[[nodiscard]] spherical_geometry::LongitudeRange longitude_range() const {
			return {
					wrap_angle(vertices[0].longitude - angular_padding),
					wrap_angle(vertices[2].longitude + angular_padding)
			};
		}

		struct TriangleEdges {
			OrderedArcEdge e_long, e_short1, e_short2;
		};

		/**
		 * Return whether the middle vertex lies above the long edge in terms of latitude.
		 */
		[[nodiscard]] bool middle_is_above() const {
			OrderedArcEdge long_edge { vertices[0], vertices[2] };

			double longedge_latitude = long_edge.latitudeAtLongitude(vertices[1].longitude);

			return longedge_latitude > vertices[1].latitude;
		}

		[[nodiscard]] TriangleEdges edges_padded() const {

			OrderedArcEdge long_edge(
					{wrap_angle(vertices[0].longitude - angular_padding), vertices[0].latitude},
					{wrap_angle(vertices[2].longitude + angular_padding), vertices[2].latitude});

			OrderedArcEdge short_edge1(
					{wrap_angle(vertices[0].longitude - angular_padding), vertices[0].latitude},
					{vertices[1].longitude, vertices[1].latitude});

			OrderedArcEdge short_edge2(
					{vertices[1].longitude, vertices[1].latitude},
					{wrap_angle(vertices[2].longitude + angular_padding), vertices[2].latitude});

			if (middle_is_above())
			{
				long_edge.start.latitude -= angular_padding;
				long_edge.end.latitude -= angular_padding;
				short_edge1.start.latitude += angular_padding;
				short_edge1.end.latitude += angular_padding;
				short_edge2.start.latitude += angular_padding;
				short_edge2.end.latitude += angular_padding;
			} else {
				long_edge.start.latitude += angular_padding;
				long_edge.end.latitude += angular_padding;
				short_edge1.start.latitude -= angular_padding;
				short_edge1.end.latitude -= angular_padding;
				short_edge2.start.latitude -= angular_padding;
				short_edge2.end.latitude -= angular_padding;
			}

			return {long_edge, short_edge1, short_edge2};
		}

		[[nodiscard]] spherical_geometry::LatitudeRange latitude_range_at_longitude(const Longitude& longitude) const {
			assert(this->longitude_range().contains(longitude));

			TriangleEdges edges = this->edges_padded();

			double lat1 = edges.e_long.latitudeAtLongitude(longitude.longitude);

			double lat2;

			if (edges.e_short1.longitude_range().contains(longitude)) {
				lat2 = edges.e_short1.latitudeAtLongitude(longitude.longitude);
			} else {
				lat2 = edges.e_short2.latitudeAtLongitude(longitude.longitude);
			}

			if (lat1 < lat2) {
				return {lat1, lat2};
			} else {
				return {lat2, lat1};
			}
		}

		/**
		 * Returns whether this triangle crosses the given latitude in the given longitude range.
		 * @param lat 			The latitude to check.
		 * @param range 		The longitude in which range to check.
		 * @return 	-1 if the triangle is before, 0 if it crosses, 1 if it is after.
		 */
		[[nodiscard]] int crosses_lat_in_longitude_range(double lat, const LongitudeRange &range) const {

//			double min_lat = std::numeric_limits<double>::infinity();
//			double max_lat = -std::numeric_limits<double>::infinity();
//
//			// Iterate over the edges and restrict them to the given longitude range.
//
//			for (int i = 0; i < 3; ++i) {
//				const auto &a = vertices[i];
//				const auto &b = vertices[(i + 1) % 3];
//
//				if (range.contains(a.longitude) && range.contains(b.longitude)) {
//					// Both vertices are in the range, so the edge is in the range.
//					min_lat = std::min(min_lat, std::min(a.latitude, b.latitude));
//					max_lat = std::max(max_lat, std::max(a.latitude, b.latitude));
//				} else {
//					double slope = (b.latitude - a.latitude) / (b.longitude - a.longitude);
//					double intercept = a.latitude - slope * a.longitude;
//
//
//				}
//			}

			throw std::runtime_error("Not implemented");

		}

		[[nodiscard]] LatitudeRange occupied_latitudes(const LongitudeRange& lon);

	};

}

#endif //MGODPL_SPHERICAL_GEOMETRY_H
