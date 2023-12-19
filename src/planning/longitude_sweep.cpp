//
// Created by werner on 28-11-23.
//

#include "longitude_sweep.h"

#include <iostream>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

constexpr double DOUBLE_EPSILON = 1e-14; // TODO: Double-check that events aren't this close together.

namespace mgodpl {
	TriangleEdges triangle_edges(const std::array<RelativeVertex, 3> &vertices) {
		return TriangleEdges{
				.short_1 = Edge{vertices[0].local_vertex, vertices[1].local_vertex},
				.short_2 = Edge{vertices[1].local_vertex, vertices[2].local_vertex},
				.long_edge = Edge{vertices[0].local_vertex, vertices[2].local_vertex}
		};
	}

	bool vertex_is_above_long_edge(const std::array<RelativeVertex, 3> &vertices) {
		// grab the latitude range at the middle vertex.
		double latitude_edge = latitude(
				Edge{vertices[0].local_vertex, vertices[2].local_vertex},
				longitude(vertices[1].local_vertex, math::Vec3d(0, 0, 0)));

		double latitude_vertex = latitude(vertices[1].local_vertex, math::Vec3d(0, 0, 0));

		return latitude_vertex > latitude_edge;
	}

	std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle &triangle, const math::Vec3d &center) {
		std::array<RelativeVertex, 3> vertices{
				RelativeVertex{
						longitude(triangle.vertices[0], center), latitude(triangle.vertices[0], center),
						triangle.vertices[0] - center
				},
				RelativeVertex{
						longitude(triangle.vertices[1], center), latitude(triangle.vertices[1], center),
						triangle.vertices[1] - center
				},
				RelativeVertex{
						longitude(triangle.vertices[2], center), latitude(triangle.vertices[2], center),
						triangle.vertices[2] - center
				}
		};

		std::sort(vertices.begin(), vertices.end(), [](const auto &a, const auto &b) {
			return signed_longitude_difference(a.longitude, b.longitude) < 0;
		});

		return vertices;
	}

	bool vertices_cross_longitude(const std::array<RelativeVertex, 3> &vertices, double longitude) {
		return signed_longitude_difference(longitude, vertices[0].longitude) >= 0 &&
			   signed_longitude_difference(longitude, vertices[2].longitude) <= 0;
	}

	double SortByLatitudeAtLongitude::latitudeAtCurrentLongitude(const OccupiedRangeBetweenEdges &a) const {
		return latitude(a.min_latitude_edge, sweep->current_sweepline_longitude());
	}

	bool SortByLatitudeAtLongitude::operator()(const OccupiedRangeBetweenEdges &a,
											   const OccupiedRangeBetweenEdges &b) const {
		return before_at_longitude(a, b, sweep->current_sweepline_longitude());
	}

	bool SortByLatitudeAtLongitude::before_at_longitude(const OccupiedRangeBetweenEdges &a,
														const OccupiedRangeBetweenEdges &b,
														const double longitude) const {

		double l1 = latitude(a.min_latitude_edge, longitude);
		double l2 = latitude(b.min_latitude_edge, longitude);

		if (l1 != l2) {
			return l1 < l2;
		} else {
			// Sort by end of shared longitude range:
			double max_longitude = signed_longitude_difference(a.max_longitude, b.max_longitude) < 0 ? a.max_longitude
																									 : b.max_longitude;
			double l1_max = latitude(a.min_latitude_edge, max_longitude);
			double l2_max = latitude(b.min_latitude_edge, max_longitude);
			return l1_max < l2_max;
		}

	}

	double signed_longitude_difference(double first, double second) {
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
	 * \brief Test whether the given longitude is in the range of the given edge.
	 * \param edge The edge to project onto the sphere.
	 * \param lon The longitude of the sweep arc.
	 * \return True if the arc intersects the projected edge.
	 */
	bool in_longitude_range(const Edge &edge, double lon) {
		return signed_longitude_difference(lon, longitude(edge.vertices[0], math::Vec3d(0, 0, 0))) >= 0 &&
			   signed_longitude_difference(lon, longitude(edge.vertices[1], math::Vec3d(0, 0, 0))) <= 0;
	}

	math::Vec3d Edge_intersection(const Edge &a, const Edge &b) {
		// We can actually reduce this problem to linear algebra.

		// Consider the open triangle Ta formed by the two rays from (0,0,0) through a[0] and a[1],
		// and the open triangle Tb formed by the two rays from (0,0,0) through b[0] and b[1].

		// We'd now like to compute the ray of intersection of Ta and Tb, if it exists.

		// This can be done by computing the intersection of the planes containing Ta and Tb, and then
		// intersecting that line with the unit sphere.

		// Let's compute the two plane normals:

		math::Vec3d nA = a.vertices[0].cross(a.vertices[1]);
		math::Vec3d nB = b.vertices[0].cross(b.vertices[1]);

		math::Vec3d dir = nA.cross(nB).normalized();

		// Take the dot product with the first vertex of a to see if this is the antipode of the intersection.
		if (dir.dot(a.vertices[0]) < 0) {
			// Sanity check: dot product with the other 3 should also be negative.
			assert(dir.dot(a.vertices[1]) < 0);
			assert(dir.dot(b.vertices[0]) < 0);
			assert(dir.dot(b.vertices[1]) < 0);

			dir = -dir;
		}

		// The direction vector of the intersection line is the cross product of the two plane normals.
		// Return the longitude.
		return dir;
	}

	void LongitudeSweep::register_edge_pair(const OccupiedRangeBetweenEdges &range) {

		std::cerr << "Registering edge pair." << std::endl;

		// Will we encounter the start of this edge in the future?

		// If so, throw in a start event:
		if (range_is_ahead(range)) {
			std::cerr << "Registering future range." << std::endl;
			register_future_range(range);
		}

		// Are we before the end? (This can coincide with being before the start if the edge crosses the starting longitude)
		// If so, insert it into the ongoing ranges:
		if (current_sweepline_crosses_range(range)) {
			std::cerr << "Registering ongoing range." << std::endl;

			// Check for intersections:
			double max_longitude = range.max_longitude;


			if (!ranges.empty()) {


				auto lower_bound = ranges.lower_bound(range);
				auto upper_bound = ranges.upper_bound(range);

				// Find the first longitude at which an intersection occurs.

				// First, check if there might be an intersection with the previous range:
				if (lower_bound != ranges.begin()) {
					std::cerr << "Checking for intersection with previous range." << std::endl;

					// lower_bound is defined a bit weirdly, so we need to decrement it first.
					//
					// https://cplusplus.com/reference/set/set/lower_bound/:
					// "Returns an iterator pointing to the first element in the container which is not considered to go before val (i.e., either it is equivalent or goes after)."

					assert(lower_bound == ranges.end() || !ranges.key_comp().operator()(*lower_bound, range));
					--lower_bound;
					assert(ranges.key_comp().operator()(*lower_bound, range));
					if (auto intersection = edges_will_cross(*lower_bound, range)) {
						std::cerr << "Found intersection with previous range." << std::endl;
						max_longitude = *intersection;
					} else {
						std::cerr << "No intersection with previous range:" << lower_bound->min_longitude << " - "
								  << lower_bound->max_longitude << std::endl;
					}
				} else {

					// Print the guard conditions:
					std::cerr << "No previous range because " << std::endl;
					if (lower_bound == ranges.end()) {
						std::cerr << "lower_bound == ranges.end()" << std::endl;
					}
					if (lower_bound == ranges.begin()) {
						std::cerr << "lower_bound == ranges.begin()" << std::endl;
					}

				}

				// Then check if there might be an intersection with the next range:
				if (upper_bound != ranges.end()) {
					if (auto intersection = edges_will_cross(range, *upper_bound)) {
						std::cerr << "Found intersection with next range." << std::endl;
						max_longitude = signed_longitude_difference(*intersection, max_longitude) < 0 ? *intersection
																									  : max_longitude;
					} else {
						std::cerr << "No intersection with next range." << std::endl;
					}
				}
			}

			// Make sure it's ahead of the current sweepline:
			assert(signed_longitude_difference(max_longitude, current_longitude) > 0);

			// If there was an intersection, split the range.
			if (max_longitude != range.max_longitude) {
				// Split the range:
				const auto &[before, after] = range.split(max_longitude);
				register_ongoing_range(before); // Register the safe part.
				register_future_range(after); // Register the unsafe part for the future.
			} else {
				register_ongoing_range(range); // Register the entire range as safe.
			}
		}
	}

	void LongitudeSweep::register_ongoing_range(const OccupiedRangeBetweenEdges &ongoing_range) {

		assert(current_sweepline_crosses_range(ongoing_range));

		auto [it, inserted] = ranges.insert(ongoing_range);

		if (inserted) {

			if (it != ranges.begin()) {
				// Check for intersections with the previous range:
				auto prev = it;
				--prev;
//				assert(!edges_will_cross(*prev, ongoing_range));
			}

			if (std::next(it) != ranges.end()) {
				// Check for intersections with the next range:
				auto next = it;
				++next;
//				assert(!edges_will_cross(ongoing_range, *next));
			}

		}

		// Insert an end event for the safe part:
		event_queue.insert(SweepEvent(
				longitude_ahead_angle(this->starting_longitude, ongoing_range.max_longitude),
				ongoing_range.max_longitude,
				ongoing_range,
				SweepEvent::END
		));
	}

	bool
	LongitudeSweep::current_sweepline_crosses_range(const OccupiedRangeBetweenEdges &range) const {
		return signed_longitude_difference(current_longitude, range.min_longitude) >= 0 &&
			   signed_longitude_difference(current_longitude, range.max_longitude) <= 0;
	}

	bool LongitudeSweep::range_is_ahead(const OccupiedRangeBetweenEdges &range) const {
		return longitude_ahead_angle(starting_longitude, range.min_longitude) >
			   longitude_ahead_angle(starting_longitude, current_longitude);
	}

	void LongitudeSweep::register_future_range(const OccupiedRangeBetweenEdges &range) {

		assert(range_is_ahead(range));

		event_queue.insert(SweepEvent(
				longitude_ahead_angle(starting_longitude, range.min_longitude),
				range.min_longitude,
				range,
				SweepEvent::START
		));
	}

	LongitudeSweep::LongitudeSweep(const std::vector<Triangle> &triangles, double initial_longitude,
								   const math::Vec3d &center) :
			starting_longitude(initial_longitude),
			current_longitude(initial_longitude),
			ranges(SortByLatitudeAtLongitude{this}) {

		// Then, iterate over all triangles...
		for (const Triangle &triangle: triangles) {

			// If the normal is pointing towards the center, skip this triangle.
			if (triangle.normal().dot(triangle.vertices[0]) < 0) {
				continue;
			}

			std::array<RelativeVertex, 3> relative_vertices = sorted_relative_vertices(triangle, center);

			// Get the latitude of the vertices:
			double l1 = latitude(relative_vertices[0].local_vertex, math::Vec3d(0, 0, 0));
			double l2 = latitude(relative_vertices[1].local_vertex, math::Vec3d(0, 0, 0));
			double l3 = latitude(relative_vertices[2].local_vertex, math::Vec3d(0, 0, 0));

			// Skip triangles closer to the poles for now.
			if (l1 > M_PI / 3.0 && l2 > M_PI / 3.0 && l3 > M_PI / 3.0) {
				continue;
			}

			if ( l1 < -M_PI / 3.0 && l2 < -M_PI / 3.0 && l3 < -M_PI / 3.0) {
				continue;
			}

			TriangleEdges edges = triangle_edges(relative_vertices);

			OccupiedRangeBetweenEdges rg1(edges.short_1, edges.long_edge);
			OccupiedRangeBetweenEdges rg2(edges.short_2, edges.long_edge);

			if (vertex_is_above_long_edge(relative_vertices)) {
				std::swap(rg1.min_latitude_edge, rg1.max_latitude_edge);
				std::swap(rg2.min_latitude_edge, rg2.max_latitude_edge);
			}

			register_edge_pair(rg1);
			register_edge_pair(rg2);
		}

	}

	bool LongitudeSweep::check_invariants() const {

		for (const auto& range : ranges) {
			if (!current_sweepline_crosses_range(range)) {
				std::cerr << "Range is not crossed by the current sweepline!" << std::endl;
				return false;
			}
		}

		auto it = ranges.begin();
		auto it2 = it;
		++it2;

		while (it2 != ranges.end()) {

			const auto &range1 = *it;
			const auto &range2 = *it2;

			if (!ranges.key_comp()(range1, range2)) {
				std::cerr << "Ranges are not sorted!" << std::endl;
				return false;
			}

			// Check intersections:
			if (edges_will_cross(range1, range2)) {
				std::cerr << "Ranges intersect!" << std::endl;
				return false;
			}

			++it;
			++it2;
		}

		return true;

	}

	bool LongitudeSweep::check_longitude_change_is_safe(double from_longitude, double to_longitude) const {

		auto it = ranges.begin();
		auto it2 = it;
		++it2;

		while (it2 != ranges.end()) {

			const auto &range1 = *it;
			const auto &range2 = *it2;

			assert(this->ranges.key_comp().before_at_longitude(range1, range2, from_longitude));

			if (!(this->ranges.key_comp().before_at_longitude(range1, range2, to_longitude))) {

				double l1pre = latitude(range1.min_latitude_edge, from_longitude);
				double l2pre = latitude(range2.min_latitude_edge, from_longitude);

				double l1post = latitude(range1.min_latitude_edge, to_longitude);
				double l2post = latitude(range2.min_latitude_edge, to_longitude);

				auto should_have_crossed_at = this->edges_will_cross(range1, range2);

				assert(should_have_crossed_at.has_value());
				assert(should_have_crossed_at > from_longitude);
				assert(should_have_crossed_at < to_longitude);

				std::cerr << "Latitude order is not preserved!" << std::endl;
				return false;
			}

			++it;
			++it2;

		}

		return true;

	}

	std::vector<SweepEvent> LongitudeSweep::pop_events_with_same_longitude() {
		assert(!event_queue.empty());

		std::vector<SweepEvent> events{*event_queue.begin()};
		event_queue.erase(event_queue.begin());

		while (!event_queue.empty() &&
			   std::abs(event_queue.begin()->relative_longitude - events.front().relative_longitude) < DOUBLE_EPSILON) {
			events.push_back(*event_queue.begin());
			event_queue.erase(event_queue.begin());
		}

		return events;
	}

	void LongitudeSweep::progress_to_next_longitude_range() {
		// TODO: By carefully sorting the events, we should be able to avoid batching like this.

		// Grab a batch of events with the same longitude:
		std::vector<SweepEvent> events = pop_events_with_same_longitude();

		double event_longitude = events.front().longitude;

		std::cerr << "Progressing past longitude: " << event_longitude << std::endl;

		// Delete all ranges that end here:
		for (const SweepEvent &event: events) {
			if (event.event_type == SweepEvent::END) {
				ranges.erase(event.range);
			}
		}

		assert(check_invariants());
		assert(check_longitude_change_is_safe(current_longitude, event_longitude + DOUBLE_EPSILON));

		this->current_longitude = event_longitude + DOUBLE_EPSILON;

		// Check invariants:
		assert(check_invariants());

		// Then, insert all ranges that start here:
		for (const SweepEvent &event: events) {
			if (event.event_type == SweepEvent::START) {
				register_edge_pair(event.range);
			}
		}

		assert(check_invariants());

		// Put the longitude halfway to the next events:
		if (!event_queue.empty()) {
			current_longitude = (event_longitude + event_queue.begin()->longitude) / 2.0;
		}

		assert(check_longitude_change_is_safe(event_longitude, current_longitude + DOUBLE_EPSILON));
	}

	bool LongitudeSweep::has_more_events() const {
		return !event_queue.empty();
	}

	std::optional<double>
	LongitudeSweep::edges_will_cross(const OccupiedRangeBetweenEdges &range1, const OccupiedRangeBetweenEdges &range2) const {

		// If they share an endpoint, then no.
		if ((range1.min_latitude_edge.vertices[0] == range2.min_latitude_edge.vertices[0]) || (range1.min_latitude_edge.vertices[1] == range2.min_latitude_edge.vertices[1])) {
			return std::nullopt;
		}

		// Grab their shared longitude range:
		auto [start_longitude, end_longitude] = shared_longitude_range(range1, range2);
		assert(signed_longitude_difference(start_longitude, end_longitude) < 0.0);

		// Make sure that the "after_longitude" is in the right range:
		assert(signed_longitude_difference(start_longitude, current_longitude) <= 0.0);
		assert(signed_longitude_difference(current_longitude, end_longitude) <= 0.0);

		assert(ranges.key_comp().before_at_longitude(range1, range2, current_longitude));

		// If the ordering doesn't change, then return nullopt:
		if (ranges.key_comp().before_at_longitude(range1, range2, end_longitude)) {
			return std::nullopt;
		}

		if (!ranges.key_comp().before_at_longitude(range2, range1, end_longitude)) {
			double l1 = latitude(range1.min_latitude_edge, end_longitude);
			double l2 = latitude(range2.min_latitude_edge, end_longitude);


			std::cerr << "Rg1 min: " << range1.min_latitude_edge.vertices[0] << " - " << range1.min_latitude_edge.vertices[1] << std::endl;
			std::cerr << "Rg2 min: " << range2.min_latitude_edge.vertices[0] << " - " << range2.min_latitude_edge.vertices[1] << std::endl;
			std::cerr << "Ranges are equal?!" << std::endl;
		}
		assert(ranges.key_comp().before_at_longitude(range2, range1, end_longitude));

		// Else, compute the intersection:
		const math::Vec3d intersection = Edge_intersection(range1.min_latitude_edge, range2.min_latitude_edge);

		// Get the lat/lon of the intersection:
		double intersection_latitude = latitude(intersection);
		double intersection_longitude = longitude(intersection);

		// Sanity check: make sure that the intersection is in the longitude range:
		assert(signed_longitude_difference(start_longitude, intersection_longitude) < 1e-6);
		assert(signed_longitude_difference(intersection_longitude, end_longitude) < 1e-6);

		if (signed_longitude_difference(intersection_longitude, end_longitude) >= 0) {
			return std::nullopt;
		}

		// Check that the latitude of both is the intersection latitude:
		double lat1 = latitude(range1.min_latitude_edge, intersection_longitude);
		double lat2 = latitude(range2.min_latitude_edge, intersection_longitude);

		assert(std::abs(lat1 - intersection_latitude) < 1e-10);
		assert(std::abs(lat2 - intersection_latitude) < 1e-10);

		assert(ranges.key_comp().before_at_longitude(range2, range1, intersection_longitude+DOUBLE_EPSILON));

		return intersection_longitude;

	}

	double longitude_ahead_angle(const double starting_longitude, const double longitude) {
		double a_longitude = signed_longitude_difference(longitude, starting_longitude);
		if (a_longitude < 0) a_longitude += 2 * M_PI;
		return a_longitude;
	}

	math::Vec3d Triangle::normal() const {
		return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
	}

	double latitude(const math::Vec3d &point, const math::Vec3d &center) {
		math::Vec3d delta = point - center;

		// Distance from the vertical axis through the center of the sphere.
		const double distance_xy = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

		// Compute the latitude.
		return std::atan2(delta.z(), distance_xy);
	}

	double longitude(const math::Vec3d &point, const math::Vec3d &center) {
		math::Vec3d delta = point - center;

		return std::atan2(delta.y(), delta.x());
	}

	double angular_padding(double arm_radius, double obstacle_distance) {
		return atan(arm_radius / obstacle_distance);
	}

	double latitude(const Edge &edge, double at_longitude) {

		double l1 = longitude(edge.vertices[0]);
		double l2 = longitude(edge.vertices[1]);
		// Sanity check: grab the longitude range and make sure the arc is on the edge:
		assert(signed_longitude_difference(l1, at_longitude) <= 1e-10);
		assert(signed_longitude_difference(l2, at_longitude) >= -1e-10);

		// Just like intersection_longitude, we can reduce this problem to linear algebra.

		// Compute the edge plane normal.
		math::Vec3d edge_normal = edge.vertices[0].cross(edge.vertices[1]);

		math::Vec3d lon_direction = math::Vec3d(
				cos(at_longitude + M_PI / 2.0),
				sin(at_longitude + M_PI / 2.0),
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

	bool edges_will_cross(const Edge &a, const Edge &b) {

//		double start_longitude_1 = longitude(a.vertices[0], math::Vec3d(0, 0, 0));
//		double start_longitude_2 = longitude(b.vertices[0], math::Vec3d(0, 0, 0));

		double end_longitude_1 = longitude(a.vertices[1], math::Vec3d(0, 0, 0));
		double end_longitude_2 = longitude(b.vertices[1], math::Vec3d(0, 0, 0));

		double end_longitude = signed_longitude_difference(end_longitude_1, end_longitude_2) < 0
							   ? end_longitude_1
							   : end_longitude_2;
		// Sanity check: make sure that the end longitude is in the longitude range of both edges:
		assert(in_longitude_range(a, end_longitude));
		assert(in_longitude_range(b, end_longitude));

//		double start_longitude = signed_longitude_difference(start_longitude_1, start_longitude_2) > 0
//								   ? start_longitude_1
//								   : start_longitude_2;

//		double start_latitude_1 = latitude(a, start_longitude);
//		double start_latitude_2 = latitude(b, start_longitude);


		// Grab the latitude at that point:
		double latitude_1 = latitude(a, end_longitude);
		double latitude_2 = latitude(b, end_longitude);

		bool will_cross = latitude_1 > latitude_2;
//
//		assert(start_latitude_1 < start_latitude_2);





		return will_cross;
	}

	std::array<double, 2>
	shared_longitude_range(const OccupiedRangeBetweenEdges &range1, const OccupiedRangeBetweenEdges &range2) {

		double start_longitude = signed_longitude_difference(range1.min_longitude, range2.min_longitude) > 0.0 ?
								 range1.min_longitude : range2.min_longitude;

		double end_longitude = signed_longitude_difference(range1.max_longitude, range2.max_longitude) < 0.0 ?
							   range1.max_longitude : range2.max_longitude;

		// Assert that the range is non-empty.
		assert(signed_longitude_difference(end_longitude, start_longitude) >= 0.0);

		return {start_longitude, end_longitude};

	}

	bool edge_is_ordered(const Edge &edge) {
		double lon1 = longitude(edge.vertices[0], math::Vec3d(0, 0, 0));
		double lon2 = longitude(edge.vertices[1], math::Vec3d(0, 0, 0));
		return signed_longitude_difference(lon1, lon2) < 0;
	}

	std::array<double, 2> shared_longitude_range(const Edge &edge1, const Edge &edge2) {

		assert(edge_is_ordered(edge1));
		assert(edge_is_ordered(edge2));

		// Compute the longitudes of the edges:
		double start_longitude1 = longitude(edge1.vertices[0]);
		double start_longitude2 = longitude(edge2.vertices[0]);

		double end_longitude1 = longitude(edge1.vertices[1]);
		double end_longitude2 = longitude(edge2.vertices[1]);

		// Compute the shared range:
		double start_longitude =
				signed_longitude_difference(start_longitude1, start_longitude2) > 0.0 ? start_longitude1
																					  : start_longitude2;
		double end_longitude =
				signed_longitude_difference(end_longitude1, end_longitude2) < 0.0 ? end_longitude1 : end_longitude2;

		assert(signed_longitude_difference(end_longitude, start_longitude) > 0.0);

		return {start_longitude, end_longitude};
	}

	std::array<OccupiedRangeBetweenEdges, 2> OccupiedRangeBetweenEdges::split(double split_longitude) const {

		assert(signed_longitude_difference(split_longitude, min_longitude) > 0.0);
		assert(signed_longitude_difference(split_longitude, max_longitude) < 0.0);

		return {
				OccupiedRangeBetweenEdges{
						min_latitude_edge,
						max_latitude_edge,
						min_longitude,
						split_longitude
				},
				OccupiedRangeBetweenEdges{
						min_latitude_edge,
						max_latitude_edge,
						split_longitude,
						max_longitude
				}
		};
	}

	OccupiedRangeBetweenEdges::OccupiedRangeBetweenEdges(Edge min_latitude_edge, Edge max_latitude_edge)
			: min_latitude_edge(min_latitude_edge), max_latitude_edge(max_latitude_edge) {

		// Check that the edges are ordered right:
		assert(edge_is_ordered(min_latitude_edge));
		assert(edge_is_ordered(max_latitude_edge));

		// Get their shared longitude:
		auto [start_longitude, end_longitude] = shared_longitude_range(min_latitude_edge, max_latitude_edge);

		this->min_longitude = start_longitude;
		this->max_longitude = end_longitude;

	}

	OccupiedRangeBetweenEdges::OccupiedRangeBetweenEdges(Edge min_latitude_edge,
														 Edge max_latitude_edge,
														 double min_longitude,
														 double max_longitude)
			: min_latitude_edge(min_latitude_edge),
			  max_latitude_edge(max_latitude_edge),
			  min_longitude(min_longitude),
			  max_longitude(max_longitude) {

		assert(edge_is_ordered(min_latitude_edge));
		assert(edge_is_ordered(max_latitude_edge));

		auto [edge_min_longitude, edge_max_longitude] = shared_longitude_range(min_latitude_edge, max_latitude_edge);

		// Make sure the restrictions are both in the range and in the right order:
		assert(signed_longitude_difference(edge_min_longitude, min_longitude) <= 0.0);
		assert(signed_longitude_difference(max_longitude, edge_max_longitude) <= 0.0);
		assert(signed_longitude_difference(min_longitude, max_longitude) <= 0.0);

	}
}
