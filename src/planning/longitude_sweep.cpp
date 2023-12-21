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

		std::sort(vertices.begin(), vertices.end(), [](const auto &a, const auto &b) {
			return signed_longitude_difference(a.longitude, b.longitude) < 0;
		});

		return vertices;
	}

	bool SortByLatitudeAtLongitude::operator()(const OrderedArcEdge &a,
											   const OrderedArcEdge &b) const {
		return compare_at_longitude(a, b, sweep->current_longitude);
	}

	bool SortByLatitudeAtLongitude::compare_at_longitude(const OrderedArcEdge &a,
														 const OrderedArcEdge &b,
														 double longitude) const {
		double l1 = a.latitudeAtLongitude(longitude);
		double l2 = b.latitudeAtLongitude(longitude);

		if (l1 != l2) {
			return l1 < l2;
		} else {
			// Compare at the end of the shared longitude range instead.
			auto lon_range = a.longitude_range().overlap(b.longitude_range());
			double l1_end = a.latitudeAtLongitude(lon_range.end);
			double l2_end = b.latitudeAtLongitude(lon_range.end);
			return l1_end < l2_end;
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
			std::cerr << "Adding edge cross event between " << edge1 << " and " << edge2 << std::endl;
			this->event_queue.insert(mkCrossEvent(edge1, edge2));
			return true;
		} else {
			return false;
		}
	}

	SweepEvent LongitudeSweep::mkCrossEvent(const OrderedArcEdge &edge1,
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
					event_queue.insert(mkEndEvent(edge));
				}

				event_queue.insert(mkStartEvent(edge));
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

	void LongitudeSweep::advance() {

		assert(check_invariants());

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

			std::cerr << "Swapping " << swap.edge1 << " and " << swap.edge2 << std::endl;

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
				std::cerr << "Before is " << before.value() << std::endl;
				if (std::prev(it1)->interior.crosses(it1->interior, current_longitude)) {
					event_queue.erase(mkCrossEvent(std::prev(it1)->interior, it1->interior));
				}
			}
			if (std::next(it2) != ranges.end()) {
				after = std::next(it2)->interior;
				std::cerr << "After is " << after.value() << std::endl;
				if (it2->interior.crosses(std::next(it2)->interior, current_longitude)) {
					event_queue.erase(mkCrossEvent(it2->interior, std::next(it2)->interior));
				}
			}

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

//			this->current_longitude = event_longitude + DOUBLE_EPSILON;

			assert(check_invariants());

		} else {

			// Process deletions first:
			for (const auto &event: events) {
				if (const auto end = std::get_if<EdgePairEnd>(&event.event)) {

					std::cerr << "Deleting " << end->edge << std::endl;

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
						std::cerr << "Not found!" << std::endl;
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

					std::cerr << "Inserting " << start->edge << std::endl;

					// Insert the range:
					auto [it, inserted] = ranges.insert({start->edge});

					if (inserted) {

						if (it != ranges.begin()) {
							add_potential_edgecross(std::prev(it)->interior, it->interior);
						}

						if (std::next(it) != ranges.end()) {
							add_potential_edgecross(it->interior, std::next(it)->interior);
						}

						// Delete any edgecrosses with the previous/next ranges:
						if (it != ranges.begin() && std::next(it) != ranges.end()) {
							if (std::prev(it)->interior.crosses(std::next(it)->interior, current_longitude)) {
								event_queue.erase(mkCrossEvent(std::prev(it)->interior, std::next(it)->interior));
							}
						}

						// Add an end event.
						SweepEvent end_event = mkEndEvent(start->edge);

						std::cerr << "End is at longitude " << end_event.longitude << std::endl;

						this->event_queue.insert(end_event);

					}
				}
			}
		}

		assert(this->current_longitude > event_longitude);
		assert(check_invariants());

		this->events_passed += events.size();
		std::cerr << "Events passed: " << this->events_passed << ", queue size: " << this->event_queue.size() << std::endl;

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
					if (event_queue.find(evt) == event_queue.end()) {
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

		for (const auto &event: event_queue) {

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

				// There may be no coinciding deletions involving these ranges that occur before the swap.
				SweepEvent bad_deletion1 = mkEndEvent(swap->edge1);
				SweepEvent bad_deletion2 = mkEndEvent(swap->edge2);

				if (bad_deletion1.relative_longitude <= event.relative_longitude &&
					event_queue.find(bad_deletion1) != event_queue.end()) {
					std::cerr << "Bad deletion 1!" << std::endl;
					return false;
				}

				if (bad_deletion2.relative_longitude <= event.relative_longitude &&
					event_queue.find(bad_deletion2) != event_queue.end()) {
					std::cerr << "Bad deletion 2!" << std::endl;
					return false;
				}
			}
		}
		return true;
	}

	bool LongitudeSweep::check_invariants() {
		return check_order_correctness() && check_events_complete() && check_events_consistent();
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

}
