//
// Created by werner on 28-11-23.
//

#include "longitude_sweep.h"

#include <iostream>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>
#include <optional>

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

	double SortByLatitudeAtLongitude::latitudeAtCurrentLongitude(const LatitudeRangeBetweenEdges &a) const {
		return latitude(a.min_latitude_edge, sweep->current_longitude);
	}

	bool SortByLatitudeAtLongitude::operator()(const LatitudeRangeBetweenEdges &a,
											   const LatitudeRangeBetweenEdges &b) const {
		return compare_at_longitude(a, b, sweep->current_longitude);
	}

	bool SortByLatitudeAtLongitude::compare_at_longitude(const LatitudeRangeBetweenEdges &a,
														 const LatitudeRangeBetweenEdges &b,
														 double longitude) const {
		return latitude(a.min_latitude_edge, longitude) < latitude(b.min_latitude_edge, longitude);
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

	bool LongitudeSweep::add_potential_edgecross(const LatitudeRangeBetweenEdges range1,
												 const LatitudeRangeBetweenEdges range2) {

		if (edges_will_cross(range1.min_latitude_edge, range2.min_latitude_edge)) {

			auto event = mkCrossEvent(range1, range2);

			std::cerr << "Added intersection event between "
					  << range1.min_latitude_edge.vertices[0] << " -> " << range1.min_latitude_edge.vertices[1]
					  << " and "
					  << range2.min_latitude_edge.vertices[0] << " -> " << range2.min_latitude_edge.vertices[1]
					  << " at longitude " << event.longitude << std::endl;

			if
// tODO: Move this to the guard maybe
					(in_longitude_range(range1.max_latitude_edge, event.longitude)
					 && in_longitude_range(range2.max_latitude_edge, event.longitude)
					 && in_longitude_range(range1.min_latitude_edge, event.longitude)
					 && in_longitude_range(range2.min_latitude_edge, event.longitude)) {

				event_queue.insert(event);

				return true;
			} else {
				return false;
			}
		}

		return false;

	}

	SweepEvent
	LongitudeSweep::mkCrossEvent(const LatitudeRangeBetweenEdges &range1,
								 const LatitudeRangeBetweenEdges &range2) const {// Grab their intersection:
		math::Vec3d intersection = Edge_intersection(range1.min_latitude_edge, range2.min_latitude_edge);

		// Extract lat/lon:
		double intersection_longitude = longitude(intersection, math::Vec3d(0, 0, 0));

		double l1 = latitude(range1.min_latitude_edge, intersection_longitude);
		double l2 = latitude(range2.min_latitude_edge, intersection_longitude);

		assert(std::abs(l1 - l2) < DOUBLE_EPSILON);

		SweepEvent event{
				.relative_longitude = longitude_ahead_angle(starting_longitude, intersection_longitude),
				.longitude = intersection_longitude,
				.event = EdgePairSwap{range1, range2}
		};
		return event;
	}

	LongitudeSweep::LongitudeSweep(const std::vector<Triangle> &triangles, double initial_longitude,
								   const math::Vec3d &center) :
			starting_longitude(initial_longitude),
			current_longitude(initial_longitude),
			comparator{this},
			ranges(comparator) {
		// Then, iterate over all triangles...
		for (const Triangle &triangle: triangles) {

			// If the normal is pointing towards the center, skip this triangle.
			if (triangle.normal().dot(triangle.vertices[0]) < 0) {
				continue;
			}

			std::array<RelativeVertex, 3> relative_vertices = sorted_relative_vertices(triangle, center);

			TriangleEdges edges = triangle_edges(relative_vertices);

			std::array<bool, 3> vertex_passed = {
					signed_longitude_difference(current_longitude, relative_vertices[0].longitude) >= 0,
					signed_longitude_difference(current_longitude, relative_vertices[1].longitude) >= 0,
					signed_longitude_difference(current_longitude, relative_vertices[2].longitude) >= 0
			};

			bool on_first_edge = vertex_passed[0] && !vertex_passed[1];
			bool on_second_edge = vertex_passed[1] && !vertex_passed[2];
			bool on_long_edge = vertex_passed[0] && !vertex_passed[2];

			// Sanity check: on_long_edge implies on_first_edge and on_second_edge.
			assert(!on_long_edge || (on_first_edge || on_second_edge));
			// Another sanity check: can be on at most one of the short edges at once.
			assert(!(on_first_edge && on_second_edge));

			LatitudeRangeBetweenEdges rg1{
					.min_latitude_edge = edges.short_1,
					.max_latitude_edge = edges.long_edge
			};

			LatitudeRangeBetweenEdges rg2{
					.min_latitude_edge = edges.short_2,
					.max_latitude_edge = edges.long_edge
			};

			if (vertex_is_above_long_edge(relative_vertices)) {
				std::swap(rg1.min_latitude_edge, rg1.max_latitude_edge);
				std::swap(rg2.min_latitude_edge, rg2.max_latitude_edge);
			}

			bool rg1_excluded = false;

			if (on_first_edge) {
				// Check to make sure we're on the longitude range:
				assert(in_longitude_range(edges.short_1, current_longitude));
				assert(in_longitude_range(edges.long_edge, current_longitude));
				ranges.insert({rg1});
			}

			if (!rg1_excluded) {
				// Start/end events:
				event_queue.insert(mkStartEvent(rg1, relative_vertices[0].longitude));
				auto end = mkEndEvent(rg1);
				assert(end.longitude == relative_vertices[1].longitude);
				event_queue.insert(end);
			} else {
				std::cerr << "Excluded rg1" << std::endl;
			}

			bool rg2_excluded = false;

			if (on_second_edge) {
				// Check to make sure we're on the longitude range:
				assert(in_longitude_range(edges.short_2, current_longitude));
				assert(in_longitude_range(edges.long_edge, current_longitude));
				ranges.insert({rg2});
			}

			if (!rg2_excluded) {
				event_queue.insert(mkStartEvent(rg2, relative_vertices[1].longitude));
				auto end = mkEndEvent(rg2);
				assert(end.longitude == relative_vertices[2].longitude);
				event_queue.insert(end);
			} else {
				std::cerr << "Excluded rg2" << std::endl;
			}
		}

		// Then, for all the ongoing latitude ranges, register any potential switching events.
		auto it = ranges.begin();

		do {
			const auto range1 = *it;
			++it;
			if (it == ranges.end()) {
				break;
			}
			const auto range2 = *it;

			add_potential_edgecross(range1, range2);

		} while (it != ranges.end());

		assert(check_order_correctness());
		assert(check_events_consistent());
	}

	SweepEvent LongitudeSweep::mkEndEvent(const LatitudeRangeBetweenEdges &rg1) const {

		double lon1 = longitude(rg1.min_latitude_edge.vertices[1], math::Vec3d(0, 0, 0));
		double lon2 = longitude(rg1.max_latitude_edge.vertices[1], math::Vec3d(0, 0, 0));

		double min_lon = signed_longitude_difference(lon1, lon2) < 0 ? lon1 : lon2;

		return SweepEvent{
				.relative_longitude = longitude_ahead_angle(starting_longitude, min_lon),
				.longitude = min_lon,
				.event = EdgePairEnd{rg1}
		};
	}

	SweepEvent LongitudeSweep::mkStartEvent(const LatitudeRangeBetweenEdges &rg1, double longitude) const {
		return SweepEvent{
				.relative_longitude = longitude_ahead_angle(starting_longitude, longitude),
				.longitude = longitude,
				.event = EdgePairStart{rg1}
		};
	}

	std::vector<SweepEvent> LongitudeSweep::pop_next_events() {
		assert(!event_queue.empty());

		std::vector<SweepEvent> events{*event_queue.begin()};
		event_queue.erase(event_queue.begin());

		while (!event_queue.empty() && event_queue.begin()->relative_longitude == events[0].relative_longitude) {
			events.push_back(*event_queue.begin());
			event_queue.erase(event_queue.begin());
		}

		return events;
	}

	void LongitudeSweep::delete_affected_ranges(const std::vector<SweepEvent> &events) {
		// First pass: delete all affected ranges:
		for (const auto &event: events) {
			switch (event.event.index()) {
				case 0: // Range start.
					ranges.erase(std::get<EdgePairStart>(event.event).range);
					break;
				case 1: // Range end.
					// Just delete it.
					ranges.erase(std::get<EdgePairEnd>(event.event).range);
					break;
				case 2: // Range swap.
					// Also just delete them:
					ranges.erase(std::get<EdgePairSwap>(event.event).range1);
					ranges.erase(std::get<EdgePairSwap>(event.event).range2);
					break;
				default:
					throw std::runtime_error("Invalid event type");
			}
		}
	}

	void LongitudeSweep::reinsert_nondeleted(const std::vector<SweepEvent> &events) {
		// Reinsert all except the deleted ranges.
		for (const auto &event: events) {
			switch (event.event.index()) {
				case 0: // Range start.
					ranges.insert(std::get<EdgePairStart>(event.event).range);
					break;
				case 1: // Range end.
					// Do nothing.
					break;
				case 2: // Range swap.
					ranges.insert(std::get<EdgePairSwap>(event.event).range1);
					ranges.insert(std::get<EdgePairSwap>(event.event).range2);
					break;
				default:
					throw std::runtime_error("Invalid event type");
			}
		}
	}

	void LongitudeSweep::process_next_event() {

		// We're going to try to increase the longitude of the sweep arc.

		// First, let's check out invariants:
		assert(check_order_correctness());
		// We haen't changed anything, but this doesn't hurt.
		assert(check_events_complete());

		// Then, grab all events that occur at this longitude.
		const auto events = pop_next_events();

		// Extract the longitude of the events:
		double event_longitude = events[0].longitude;

		std::cerr << "Processing events at longitude " << event_longitude << std::endl;

		bool additions = std::any_of(events.begin(), events.end(), [](const SweepEvent &evt) {
			return std::get_if<EdgePairStart>(&evt.event) != nullptr;
		});

		bool swaps = std::any_of(events.begin(), events.end(), [](const SweepEvent &evt) {
			return std::get_if<EdgePairSwap>(&evt.event) != nullptr;
		});

		bool deletions = std::any_of(events.begin(), events.end(), [](const SweepEvent &evt) {
			return std::get_if<EdgePairEnd>(&evt.event) != nullptr;
		});

		assert((additions || deletions) != swaps);

		if (swaps) {

			assert(events.size() == 1);

			const auto &swap = std::get<EdgePairSwap>(events[0].event);

			std::cerr << "Swapping "
					  << swap.range1.min_latitude_edge.vertices[0] << " -> "
					  << swap.range1.min_latitude_edge.vertices[1] << " and "
					  << swap.range2.min_latitude_edge.vertices[0] << " -> "
					  << swap.range2.min_latitude_edge.vertices[1] << std::endl;

			// Find the ranges:
			auto it1 = ranges.find(swap.range1);
			auto it2 = ranges.find(swap.range2);

			// Check if they're there:
			assert(it1 != ranges.end());
			assert(it2 != ranges.end());

			// And that they're next to each other:
			assert(std::next(it1) == it2);

			std::optional<LatitudeRangeBetweenEdges> neigbor1, neigbor2;

			// If they have previous/next neighbors, delete any potential edgecrosses with them:
			if (it1 != ranges.begin()) {
				neigbor1 = *std::prev(it1);
				SweepEvent prev_swap_event = mkCrossEvent(*std::prev(it1), *it1);
				event_queue.erase(prev_swap_event);
			}

			if (std::next(it2) != ranges.end()) {
				neigbor2 = *std::next(it2);
				SweepEvent next_swap_event = mkCrossEvent(*it2, *std::next(it2));
				event_queue.erase(next_swap_event);
			}

			// Sanity check: are the neighbors actually in order?
			if (neigbor1 && neigbor2) {
				assert(ranges.key_comp()(*neigbor1, *neigbor2));
			}

			// Delete them:
			ranges.erase(swap.range1);
			ranges.erase(swap.range2);

			assert(check_order_correctness());

			// Sanity check: are the neighbors actually in order?
			if (neigbor1 && neigbor2) {
				assert(ranges.key_comp()(*neigbor1, *neigbor2));

				add_potential_edgecross(*neigbor1, *neigbor2);
			}

			assert(check_events_complete());
			assert(check_events_consistent());
			assert(check_order_correctness());

			// Advance to halfway to the current event:
			this->current_longitude = (this->current_longitude + event_longitude) / 2;

			assert(check_events_complete());
			assert(check_events_consistent());
			assert(check_order_correctness());
			assert(jump_is_safe(event_longitude + DOUBLE_EPSILON));

			// Advance the longitude:
			this->current_longitude = event_longitude + DOUBLE_EPSILON;

			assert(check_events_complete());
			assert(check_events_consistent());
			assert(check_order_correctness());

			// Sanity check: are the neighbors actually in order?
			if (neigbor1 && neigbor2) {
				assert(ranges.key_comp()(*neigbor1, *neigbor2));

				this->event_queue.erase(mkCrossEvent(*neigbor1, *neigbor2));
			}

			// Reinsert them:
			ranges.insert(swap.range1);
			ranges.insert(swap.range2);

			// Find them back (iterators are disturbed by insertions):

			it1 = ranges.find(swap.range1);
			it2 = ranges.find(swap.range2);

			// Check if they're there:
			assert(it1 != ranges.end());
			assert(it2 != ranges.end());

			// And that they're next to each other:
			assert(std::next(it2) == it1);

			// Check that the neighbors are still there:
			if (neigbor1) {
				assert(ranges.find(*neigbor1) == std::prev(it2));
			}

			if (neigbor2) {
				assert(ranges.find(*neigbor2) == std::next(it1));
			}

			// Check for swap events with their new neighbors:
			if (it2 != ranges.begin()) {
				add_potential_edgecross(*std::prev(it2), *it2);
			}

			if (std::next(it1) != ranges.end()) {
				add_potential_edgecross(*it1, *std::next(it1));
			}

			// Check invariants:
			assert(check_events_consistent());
			assert(check_order_correctness());
			assert(check_events_complete());

		} else {

			// Process deletions first:
			for (const auto &event: events) {
				if (const auto end = std::get_if<EdgePairEnd>(&event.event)) {

					std::cerr << "Deleting " << end->range.min_latitude_edge.vertices[0] << " -> "
							  << end->range.min_latitude_edge.vertices[1] << std::endl;

					// Find it.
					auto it = ranges.find(end->range);

					// Check if it's there:
					if (it !=
						ranges.end()) { // Sometimes duplicate ranges are not properly inserted; so they won't be found.

						// If it's not the first or last, add the potential edgecrosses:
						if (it != ranges.begin() && std::next(it) != ranges.end()) {
							add_potential_edgecross(*std::prev(it), *std::next(it));
						}

						// Erase it.
						ranges.erase(it);
					}
				}
			}

			// Check invariants:
			assert(check_order_correctness());

			// Advance the longitude:
			this->current_longitude = event_longitude + DOUBLE_EPSILON;

			// Check invariants:
			assert(check_order_correctness());

			// Process insertions:
			for (const auto &event: events) {
				if (const auto start = std::get_if<EdgePairStart>(&event.event)) {

					std::cerr << "Inserting " << start->range.min_latitude_edge.vertices[0] << " -> "
							  << start->range.min_latitude_edge.vertices[1] << std::endl;

					// Insert the range:
					auto [it, inserted] = ranges.insert(start->range);

					if (inserted) {

						if (it != ranges.begin()) {
							add_potential_edgecross(*std::prev(it), *it);
						}

						if (std::next(it) != ranges.end()) {
							add_potential_edgecross(*it, *std::next(it));
						}

						// Add an end event.
						SweepEvent end_event = mkEndEvent(start->range);

						std::cerr << "End is at longitude " << end_event.longitude << std::endl;

						this->event_queue.insert(end_event);

					}
				}
			}

			// Check invariants:
			assert(check_events_complete());
			assert(check_events_consistent());
			assert(check_order_correctness());
		}

		assert(check_order_correctness());
		assert(check_events_complete());
		assert(this->current_longitude > event_longitude);
		assert(check_events_consistent());

	}

	bool LongitudeSweep::has_more_events() const {
		return !event_queue.empty();
	}

	bool LongitudeSweep::check_order_correctness() {

		// Check whether all successive pairs of ranges are ordered correctly.
		if (ranges.size() >= 2) {

			auto it1 = ranges.begin();
			auto it2 = std::next(it1);

			while (it2 != ranges.end()) {

				if (!ranges.key_comp()(*it1, *it2)) {

					std::cerr << "Not in order: " << it1->min_latitude_edge.vertices[0] << " -> "
							  << it1->min_latitude_edge.vertices[1] << " vs "
							  << it2->min_latitude_edge.vertices[0] << " -> "
							  << it2->min_latitude_edge.vertices[1] << std::endl;
					std::cerr << "Latitude at current longitude: " << ranges.key_comp().latitudeAtCurrentLongitude(*it1)
							  << " vs "
							  << ranges.key_comp().latitudeAtCurrentLongitude(*it2) << std::endl;

					bool will_cross_prediction = edges_will_cross(it2->min_latitude_edge, it1->min_latitude_edge);

					std::cerr << "Will cross prediction: " << will_cross_prediction << std::endl;

					auto event = mkCrossEvent(*it2, *it1);

					return false;
				}

				++it1;
				++it2;
			}

		}

		return true;

	}

	bool LongitudeSweep::check_events_complete() {
		// Check whether all successive pairs of ranges are ordered correctly.
		if (ranges.size() >= 2) {

			auto it1 = ranges.begin();
			auto it2 = std::next(it1);

			while (it2 != ranges.end()) {

				// Also, whether, if there's an intersection between them, that there's an edge cross event too:
				if (edges_will_cross(it1->min_latitude_edge, it2->min_latitude_edge)) {
					if (event_queue.find(mkCrossEvent(*it1, *it2)) == event_queue.end()) {
						std::cerr << "Missing edge cross event between " << it1->min_latitude_edge.vertices[0]
								  << " -> "
								  << it1->min_latitude_edge.vertices[1] << " and "
								  << it2->min_latitude_edge.vertices[0] << " -> "
								  << it2->min_latitude_edge.vertices[1] << std::endl;
						return false;
					}
				}

				++it1;
				++it2;
			}

		}

		return true;
	}

	bool LongitudeSweep::check_events_consistent() {

		for (const auto &event: event_queue) {

			// Check that the events are in the right order:
			if (const auto swap = std::get_if<EdgePairSwap>(&event.event)) {
				auto it1 = ranges.find(swap->range1);
				auto it2 = ranges.find(swap->range2);

				if (it1 == ranges.end() || it2 == ranges.end()) {
					std::cerr << "Missing range in event queue!" << std::endl;
					return false;
				}

				if (std::next(it1) != it2) {
					std::cerr << "Ranges not adjacent in event queue!" << std::endl;
					std::cerr << "Ranges are " << it1->min_latitude_edge.vertices[0] << " -> "
							  << it1->min_latitude_edge.vertices[1] << " and "
							  << it2->min_latitude_edge.vertices[0] << " -> "
							  << it2->min_latitude_edge.vertices[1] << std::endl;
					return false;
				}

				// There may be no coinciding deletions involving these ranges that occur before the swap.
				SweepEvent bad_deletion1 = mkEndEvent(swap->range1);
				SweepEvent bad_deletion2 = mkEndEvent(swap->range2);

				if (bad_deletion1.relative_longitude <= event.relative_longitude &&
					event_queue.find(bad_deletion1) != event_queue.end()) {
					std::cerr << "Bad deletion 1!" << std::endl;
					return false;
				}

				if (bad_deletion2.relative_longitude <= event.relative_longitude &&
					event_queue.find(bad_deletion2) != event_queue.end()) {
					bool cross_prediction = edges_will_cross(swap->range1.min_latitude_edge,
															 swap->range2.min_latitude_edge);

					std::cerr << "Bad deletion 2!" << std::endl;
					return false;
				}
			}
		}
		return true;
	}

	bool LongitudeSweep::jump_is_safe(double target_longitude) {

		for (const auto &range: ranges) {
			if (!in_longitude_range(range.min_latitude_edge, target_longitude)) {
				std::cerr << "Did we miss an end event?" << std::endl;
				return false;
			}
		}

		// Check the order, and check that it shall be maintained:
		if (this->ranges.empty()) {
			return true;
		}

		auto it1 = this->ranges.begin();
		auto it2 = std::next(it1);

		while (it2 != this->ranges.end()) {

			if (!this->ranges.key_comp()(*it1, *it2)) {
				std::cerr << "Not in order at start: " << it1->min_latitude_edge.vertices[0] << " -> "
						  << it1->min_latitude_edge.vertices[1] << " vs "
						  << it2->min_latitude_edge.vertices[0] << " -> "
						  << it2->min_latitude_edge.vertices[1] << std::endl;
				return false;
			}

			if (!this->ranges.key_comp().compare_at_longitude(*it1, *it2, target_longitude)) {

				double l1 = latitude(it1->min_latitude_edge, target_longitude);
				double l2 = latitude(it2->min_latitude_edge, target_longitude);

				std::cerr << "This is wrong; did we not predict a swap?" << std::endl;
				bool swap_prediction = edges_will_cross(it1->min_latitude_edge, it2->min_latitude_edge);

				SweepEvent swap = mkCrossEvent(*it1, *it2);

				std::cerr << "Not in order at target: " << it1->min_latitude_edge.vertices[0] << " -> "
						  << it1->min_latitude_edge.vertices[1] << " vs "
						  << it2->min_latitude_edge.vertices[0] << " -> "
						  << it2->min_latitude_edge.vertices[1] << std::endl;
				return false;
			}

			++it1;
			++it2;

		}

		return true;

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

		// If they have a shared endpoint, they won't cross.
		if (a.vertices[0] == b.vertices[0] || a.vertices[1] == b.vertices[1]) { // FIXME; this is wrong?!
			return false;
		}

		double end_longitude_1 = longitude(a.vertices[1], math::Vec3d(0, 0, 0));
		double end_longitude_2 = longitude(b.vertices[1], math::Vec3d(0, 0, 0));

		double end_longitude = signed_longitude_difference(end_longitude_1, end_longitude_2) < 0
							   ? end_longitude_1
							   : end_longitude_2;

		// Sanity check: make sure that the end longitude is in the longitude range of both edges:
		assert(in_longitude_range(a, end_longitude));
		assert(in_longitude_range(b, end_longitude));

		// Check to make sure the end longitudes are different:
		if (std::abs(signed_longitude_difference(end_longitude_1, end_longitude_2)) < DOUBLE_EPSILON) {
			return false;
		}

		// Grab the latitude at that point:
		double latitude_1 = latitude(a, end_longitude);
		double latitude_2 = latitude(b, end_longitude);

		bool will_cross = latitude_1 > latitude_2;

		return will_cross;
	}

	bool cmp_ranges_holdbuffer(const LatitudeRangeBetweenEdges &a, const LatitudeRangeBetweenEdges &b) {
		if (a.min_latitude_edge.vertices[0] == b.min_latitude_edge.vertices[0]) {
			return a.min_latitude_edge.vertices[1] < b.min_latitude_edge.vertices[1];
		} else {
			return a.min_latitude_edge.vertices[0] < b.min_latitude_edge.vertices[0];
		}
	}
}
