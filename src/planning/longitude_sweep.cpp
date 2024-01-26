//
// Created by werner on 28-11-23.
//

#include "longitude_sweep.h"
#include "geometry.h"

#include <iostream>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>
#include <optional>

constexpr double DOUBLE_EPSILON = 1e-14; // TODO: Double-check that events aren't this close together.

#define LOG_TRACE_BEFORE_SWAP 0 // NOLINT(*-macro-to-enum)
#define LOG_TRACE_AFTER_SWAP 0 // NOLINT(*-macro-to-enum)
#define TRACE_SWAP_EVENTS 0 // NOLINT(*-macro-to-enum)
#define EXPENSIVE_CHECK_INVARIANTS 0 // NOLINT(*-macro-to-enum)

namespace mgodpl {
	using namespace spherical_geometry;
//
//	std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle &triangle, const math::Vec3d &center) {
//
//		std::array<RelativeVertex, 3> vertices{
//				RelativeVertex {
//						longitude(triangle.vertices[0], center), latitude(triangle.vertices[0], center),
//				},
//				RelativeVertex{
//						longitude(triangle.vertices[1], center), latitude(triangle.vertices[1], center),
//				},
//				RelativeVertex{
//						longitude(triangle.vertices[2], center), latitude(triangle.vertices[2], center),
//				}
//		};
//
//		std::sort(vertices.begin(), vertices.end(), [](const auto &a, const auto &b) {
//			return signed_longitude_difference(a.longitude, b.longitude) < 0;
//		});
//
//		return vertices;
//	}

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

	bool LongitudeSweep::add_potential_edgecross(const OrderedArcEdge edge1, const OrderedArcEdge edge2) {
		if (edge1.crosses(edge2, current_longitude)) {

			auto evt = mkCrossEvent(edge1, edge2);

			assert(signed_longitude_difference(evt.longitude, current_longitude) > 0);

			if (evt.relative_longitude >= longitude_ahead_angle(starting_longitude, current_longitude)) {
				this->event_queue.insert_refindable(evt);
				return true;
			} else {
				// This edgecross is past the end of the sweep; don't add it.
				return false;
			}
		} else {
			return false;
		}
	}

	inline SweepEvent LongitudeSweep::mkCrossEvent(const OrderedArcEdge &edge1,
											const OrderedArcEdge &edge2) const {

		// Grab their intersection:
		auto intersection = edge1.intersection(edge2);

		SweepEvent event{
				.relative_longitude = longitude_ahead_angle(starting_longitude, intersection.longitude),
				.longitude = intersection.longitude,
				.event = EdgePairSwap{ edge1, edge2 }
		};

		return event;
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

			for (const auto& edge : {OrderedArcEdge(relative_vertices[0], relative_vertices[1]), OrderedArcEdge(relative_vertices[1], relative_vertices[2])}) {

				if (edge.longitude_range().contains(current_longitude)) {
					ranges.insert({edge});
					event_queue.insert_non_refindable(mkEndEvent(edge));
				}

				event_queue.insert_non_refindable(mkStartEvent(edge));
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

			add_potential_edgecross(range1.interior, range2.interior);

		} while (it != ranges.end());

		assert(check_order_correctness());
		assert(check_events_consistent());
		assert(check_events_complete());
	}

	SweepEvent LongitudeSweep::mkEndEvent(const OrderedArcEdge &rg1) const {
		return SweepEvent {
			.relative_longitude = longitude_ahead_angle(starting_longitude, rg1.end.longitude),
			.longitude = rg1.end.longitude,
			.event = EdgePairEnd{rg1}
		};
	}

	SweepEvent LongitudeSweep::mkStartEvent(const OrderedArcEdge &rg1) const {
		return SweepEvent {
			.relative_longitude = longitude_ahead_angle(starting_longitude, rg1.start.longitude),
			.longitude = rg1.start.longitude,
			.event = EdgePairStart{rg1}
		};
	}

	const std::vector<SweepEvent> & LongitudeSweep::pop_next_events() {
		assert(!event_queue.empty());

		static std::vector<SweepEvent> events;
		events.clear();

		auto rit1 = event_queue._refindable.begin();

		double rlon_refindable = (rit1 == event_queue._refindable.end()) ? INFINITY : rit1->relative_longitude;
		double rlon_non_refindable = (event_queue._non_refindable.empty()) ? INFINITY : event_queue._non_refindable.top().relative_longitude;

		if (rlon_refindable < rlon_non_refindable) {
			events.push_back(*rit1);
			event_queue._refindable.erase(rit1);
		} else {
			// Grab all at the same.
			while (!event_queue._non_refindable.empty() && event_queue._non_refindable.top().relative_longitude == rlon_non_refindable) {
				events.push_back(event_queue._non_refindable.top());
				event_queue._non_refindable.pop();
			}
		}

		return events;
	}

	void LongitudeSweep::advance() {

//		std::cout << this->current_longitude << "," << this->ranges.size() << std::endl;

#if EXPENSIVE_CHECK_INVARIANTS
		assert(check_invariants());
#endif

		// Then, grab all events that occur at this longitude.
		const auto events = pop_next_events();

		// Extract the longitude of the events:
		double event_longitude = events[0].longitude;

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

#if TRACE_SWAP_EVENTS
			std::cerr << "Swapping " << swap.edge1 << " and " << swap.edge2 << std::endl;
#endif

			std::optional<OrderedArcEdge> before, after;

			// Find the ranges:
			auto it1 = ranges.find({swap.edge1});
			auto it2 = ranges.find({swap.edge2});

			// Check if they're there:
			assert(it1 != ranges.end());
			assert(it2 != ranges.end());

			// And that they're next to each other:
			assert(std::next(it1) == it2);

			// If they have previous/next neighbors, delete any potential edgecrosses with them:
			if (it1 != ranges.begin()) {
				before = std::prev(it1)->interior;
				if (std::prev(it1)->interior.crosses(it1->interior, current_longitude)) {
					event_queue.erase_refindable(mkCrossEvent(std::prev(it1)->interior, it1->interior));
				}
			}
			if (std::next(it2) != ranges.end()) {
				after = std::next(it2)->interior;
				if (it2->interior.crosses(std::next(it2)->interior, current_longitude)) {
					event_queue.erase_refindable(mkCrossEvent(it2->interior, std::next(it2)->interior));
				}
			}

#if LOG_TRACE_BEFORE_SWAP
			std::cerr << "Before swap, ranges are :";
			if (before) {
				std::cerr << " B" << before->id;
			}
			std::cerr << " " << it1->interior.id;
			std::cerr << " " << it2->interior.id;
			if (after) {
				std::cerr << " A" << after->id;
			}
			std::cerr << std::endl;
#endif
			// Swap:
			std::swap(it1->interior, it2->interior);

			// Advance longitude:
			this->current_longitude = event_longitude + DOUBLE_EPSILON;

			// If they have neighbors, add any potential edgecrosses with them:
			// Note: we've been swapping interiors; so be careful which is which!
			if (before) {
				add_potential_edgecross(*before, it1->interior);
			}

			if (after) {
				add_potential_edgecross(it2->interior, *after);
			}

#if LOG_TRACE_AFTER_SWAP
			std::cerr << "After swap, ranges are :";
			if (before) {
				std::cerr << " B" << before->id;
			}
			std::cerr << " " << it1->interior.id;
			std::cerr << " " << it2->interior.id;
			if (after) {
				std::cerr << " A" << after->id;
			}
			std::cerr << std::endl;
#endif

		} else {

			// Process deletions first:
			for (const auto &event: events) {
				if (const auto end = std::get_if<EdgePairEnd>(&event.event)) {

					// Find it.
					auto it = ranges.find({end->edge});

					// Check if it's there:
					if (it != ranges.end()) { // Sometimes duplicate ranges are not properly inserted; so they won't be found.

						// If it's not the first or last, add the potential edgecrosses:
						if (it != ranges.begin() && std::next(it) != ranges.end()) {
							add_potential_edgecross(std::prev(it)->interior, std::next(it)->interior);
						}

						// Erase it.
						ranges.erase(it);
					} else {
//						std::cerr << "Not found!" << std::endl;
					}
				}
			}

#if EXPENSIVE_CHECK_INVARIANTS
			// Check invariants:
			assert(check_order_correctness());
#endif

			// Advance the longitude:
			this->current_longitude = event_longitude;//9539 + DOUBLE_EPSILON;

#if EXPENSIVE_CHECK_INVARIANTS
			// Check invariants:
			assert(check_order_correctness());
#endif

			// Process insertions:
			for (const auto &event: events) {
				if (const auto start = std::get_if<EdgePairStart>(&event.event)) {

					// Insert the range:
					auto [it, inserted] = ranges.insert({start->edge});

					if (inserted) {

						if (it != ranges.begin()) {
							assert(signed_longitude_difference(this->event_queue.peek_first().longitude, current_longitude) > 0);
							add_potential_edgecross(std::prev(it)->interior, it->interior);
							assert(signed_longitude_difference(this->event_queue.peek_first().longitude, current_longitude) > 0);
						}

						if (std::next(it) != ranges.end()) {
							assert(signed_longitude_difference(this->event_queue.peek_first().longitude, current_longitude) > 0);
							add_potential_edgecross(it->interior, std::next(it)->interior);
							assert(signed_longitude_difference(this->event_queue.peek_first().longitude, current_longitude) > 0);
						}

						// Delete any edgecrosses with the previous/next ranges:
						if (it != ranges.begin() && std::next(it) != ranges.end()) {
							if (std::prev(it)->interior.crosses(std::next(it)->interior, current_longitude)) {
								event_queue.erase_refindable(mkCrossEvent(std::prev(it)->interior, std::next(it)->interior));
							}
						}

						// Add an end event.
						SweepEvent end_event = mkEndEvent(start->edge);

						// Don't enqueue events that are past the end of the sweep (we only cycle once)
						if (end_event.relative_longitude >= longitude_ahead_angle(starting_longitude, current_longitude)) {
							this->event_queue.insert_non_refindable(end_event);
						}
					}
				}
			}

#if EXPENSIVE_CHECK_INVARIANTS
			assert(check_invariants());
#endif

		}

		// Move current longitude to halfway to the next event.
		if (!event_queue.empty()) {
			assert(signed_longitude_difference(this->event_queue.peek_first().longitude, current_longitude) > 0);
			this->current_longitude = interpolate_longitude(event_longitude, event_queue.peek_first().longitude, 0.5);
		}

		assert(this->event_queue.empty() || signed_longitude_difference(this->event_queue.peek_first().longitude, current_longitude) > 0);

#if EXPENSIVE_CHECK_INVARIANTS
		assert(check_invariants());
#endif

		this->events_passed += events.size();

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

					double l1 = it1->interior.latitudeAtLongitude(current_longitude);
					double l2 = it2->interior.latitudeAtLongitude(current_longitude);

					auto lon_range = it1->interior.longitude_range().overlap(it2->interior.longitude_range());

					double l1_end = it1->interior.latitudeAtLongitude(lon_range.end);
					double l2_end = it2->interior.latitudeAtLongitude(lon_range.end);

					std::cerr << "Not in order: " << it1->interior << " vs " << it2->interior << std::endl;
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
				if (it1->interior.crosses(it2->interior, current_longitude)) {
					auto evt = mkCrossEvent(it1->interior, it2->interior);

					if (!event_queue.contains_refindable(evt) && evt.relative_longitude > longitude_ahead_angle(starting_longitude, current_longitude)) {
						std::cerr << "Missing edge cross event between " << it1->interior << " and " << it2->interior << std::endl;
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

		if (event_queue.empty()) {
			return true;
		}

		for (const auto &event: event_queue.refindable()) {

			// Event must be in the future:
			assert(event.relative_longitude > longitude_ahead_angle(starting_longitude, current_longitude));

			// Check that the events are in the right order:
			if (const auto swap = std::get_if<EdgePairSwap>(&event.event)) {

				auto it1 = ranges.find({swap->edge1});
				auto it2 = ranges.find({swap->edge2});

				if (it1 == ranges.end() || it2 == ranges.end()) {
					std::cerr << "Missing range in event queue!" << std::endl;
					return false;
				}

				if (std::next(it1) != it2) {
					std::cerr << "Ranges not adjacent in event queue!" << std::endl;
					std::cerr << "Ranges are " << it1->interior << " and " << it2->interior << std::endl;
					return false;
				}

//				// There may be no coinciding deletions involving these ranges that occur before the swap.
//				SweepEvent bad_deletion1 = mkEndEvent(swap->edge1);
//				SweepEvent bad_deletion2 = mkEndEvent(swap->edge2);
//
//				if (bad_deletion1.relative_longitude <= event.relative_longitude &&
//					event_queue.slow_contains_nonrefindable(bad_deletion1)) {
//					std::cerr << "Bad deletion 1!" << std::endl;
//					return false;
//				}
//
//				if (bad_deletion2.relative_longitude <= event.relative_longitude &&
//					event_queue.slow_contains_nonrefindable(bad_deletion2)) {
//					std::cerr << "Bad deletion 2!" << std::endl;
//					return false;
//				}
			} else {
				// Wrong event type.
				return false;
			}
		}
		return true;
	}

	bool LongitudeSweep::check_invariants() {
		return check_order_correctness() && check_events_complete() && check_events_consistent();
	}
}
