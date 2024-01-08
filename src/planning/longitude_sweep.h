//
// Created by werner on 28-11-23.
//

#ifndef LATITUDE_SWEEP_H
#define LATITUDE_SWEEP_H

#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <variant>

#include "../math/Vec3.h"
#include "../experiment_utils/DualPriorityQueue.h"
#include "spherical_geometry.h"
#include "geometry.h"

namespace mgodpl {

	class LongitudeSweep;

	/**
	* \brief		Compute the intersection of two polar line segments.
	*
	* Implementation based on https://math.stackexchange.com/a/3462227
	*
	* \param a		A unit vector representing the first polar line segment.
	* \param b		The second polar line segment.
	* \return
	*/
	math::Vec3d Edge_intersection(const Edge &a, const Edge &b);
//
//	/**
//	* \brief Preprocess a triangle by sorting its vertices by longitude and computing their relative coordinates to the center.
//	* \param triangle  The original triangle, in absolute coordinates.
//	* \param center    The center of the sphere.
//	* \return          The preprocessed triangle.
//	*/
//	std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle &triangle, const math::Vec3d &center);

	/**
	* \brief A struct representing an event where the sweep arc passes the start of a LatitudeRangeBetweenEdges.
	*/
	struct EdgePairStart {
		spherical_geometry::OrderedArcEdge edge;
	};

	/**
	* \brief A struct representing an event where the sweep arc passes the end of a LatitudeRangeBetweenEdges.
	*/
	struct EdgePairEnd {
		spherical_geometry::OrderedArcEdge edge;
	};

	/**
	* \brief A struct representing an event where the sweep arc passes the point where two LatitudeRangeBetweenEdges swap in the order.
	*/
	struct EdgePairSwap {
		spherical_geometry::OrderedArcEdge edge1, edge2;
	};

	/**
	* \brief An event encountered during the longitude sweep.
	*/
	struct SweepEvent {
		/// The longitude of the event, relative to the starting longitude of the sweep.
		double relative_longitude;

		/// The longitude of the event.
		double longitude;

		/// The type of event.
		std::variant<EdgePairStart, EdgePairEnd, EdgePairSwap> event;

		bool operator<(const SweepEvent &other) const {
			// First, use relative longitude.
			if (relative_longitude != other.relative_longitude) {
				return relative_longitude < other.relative_longitude;
			}

			// Else, use event type; end events come first to avoid duplicate edges.
			if (event.index() != other.event.index()) {
				return event.index() < other.event.index();
			}

			// Else, distinguish by event type.
			switch (event.index()) {
				case 0: // EdgePairStart
					return std::get<EdgePairStart>(event).edge.id < std::get<EdgePairStart>(other.event).edge.id;
				case 1: // EdgePairEnd
					return std::get<EdgePairEnd>(event).edge.id < std::get<EdgePairEnd>(other.event).edge.id;
				case 2: // EdgePairSwap
					return std::get<EdgePairSwap>(event).edge1.id < std::get<EdgePairSwap>(other.event).edge1.id;
				default:
					throw std::runtime_error("Invalid event type");
			}
		}

		inline bool operator>(const SweepEvent &other) const {
			return other < *this;
		}
	};

	template<typename T>
	struct HackyMutable {
		mutable T interior;
	};

	/**
	 * \brief A comparator that takes a mutable (!) longitude and compares two edges by their latitude at that longitude.
	 *
	 * At first glance, one might think it ill-advised to use a mutable comparator. One might be right.
	 *
	 * That said, we are trying to maintain an order as the sweep arc moves, which means that
	 * the order of intersected edges will change as the sweep arc moves. As a result, we kinda *have* to do this,
	 * and carefully maintain the datastructure so that the order of elements *within* the datastructure is is always
	 * correct.
	 */
	struct SortByLatitudeAtLongitude {
		LongitudeSweep *sweep;

		inline bool operator()(const spherical_geometry::OrderedArcEdge &a, const spherical_geometry::OrderedArcEdge &b) const;

		inline bool compare_at_longitude(const spherical_geometry::OrderedArcEdge &a,
															 const spherical_geometry::OrderedArcEdge &b,
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

		bool operator()(const HackyMutable<spherical_geometry::OrderedArcEdge> &a,
						const HackyMutable<spherical_geometry::OrderedArcEdge> &b) const {
			return operator()(a.interior, b.interior);
		}
	};

	/**
	* \brief A struct tracking the state of an ongoing longitude sweep.
	*/
	class LongitudeSweep {

		friend struct SortByLatitudeAtLongitude;

		/// The longitude of the sweep at initialization. (In range [-pi, pi])
		const double starting_longitude;

		/// A longitude after the last-passed event, but before that of the next,
		/// or the starting longitude if no events have been processed yet. (In range [-pi, pi])
		double current_longitude;

		/// The ranges of latitudes between edges between `longitude` and the next event (or the end of the sweep).
		/// Warning: this set has a *mutable comparator* that uses `current_longitude`; be very careful when changing it.
		///
		/// The core trick is to make sure that, whenever the longitude changes, the outcome of the comparator
		/// does not change between invocations. (See the invariant checks).
		std::set<HackyMutable<spherical_geometry::OrderedArcEdge>, SortByLatitudeAtLongitude> ranges;

		/// The event queue, sorted by angle ahead of the current longitude. (between 0 and 2pi)
		/// Using a std::set rather than std::priority_queue to easily detect duplicates and
		/// delete stale events.
		DualPriorityQueue<SweepEvent> event_queue;

		size_t events_passed = 0;

		/// Given two edges, check if they cross and, if so, add a EdgePairSwap event to the event queue.
		bool add_potential_edgecross(spherical_geometry::OrderedArcEdge edge1, spherical_geometry::OrderedArcEdge edge2);

		/// Due to the unstable nature of the comparator, this method will ierate through the set
		/// and check whether the order of subsequent elements is correct.
		///
		/// This operation runs in O(n) time and should only be used while debugging.
		///
		/// \return True if the checks passed.
		[[nodiscard]] bool check_order_correctness();

		/// Check whether all events that should be in the event queue are in the event queue.
		///
		/// That is:
		/// - For all ongoing edges, there is an EdgePairEnd event in the queue.
		/// - For all neighboring edges, there is an EdgePairSwap event in the queue if they cross.
		///
		/// This operation runs in O(n) time and should only be used while debugging.
		///
		/// \return True if the checks passed.
		[[nodiscard]] bool check_events_complete();

		/// Check whether all events in the event queue are consistent with the current state of the sweep.
		///
		/// That is:
		///  - For all EdgePairEnd events, the edge is in the set of ranges.
		///  - For all EdgePairSwap events, the edges are in the set of ranges, and are neighbors in order.
		///    Also, an edge may not end before a swap.
		[[nodiscard]] bool check_events_consistent();

		[[nodiscard]] bool check_invariants();

		/// Create a SweepEvent for the given edge pair swap. Note that arguments must be ordered (this is checked by assertion).
		/// Note: this method does not actually add the event to the queue. The purpose of this method is to have
		/// a consistent way of creating the events to ensure that re-creating them can be used to look them up
		/// in the event queue.
		[[nodiscard]] SweepEvent mkCrossEvent(const spherical_geometry::OrderedArcEdge &range1, const spherical_geometry::OrderedArcEdge &range2) const;

		/// Create a SweepEvent for the given edge pair start (does not add it to the queue).
		[[nodiscard]] SweepEvent mkStartEvent(const spherical_geometry::OrderedArcEdge &rg1) const;

		/// Create a SweepEvent for the given edge pair end (does not add it to the queue).
		[[nodiscard]] SweepEvent mkEndEvent(const spherical_geometry::OrderedArcEdge &rg1) const;

		/// Remove all events fro the front of the event queue that have the same longitude.
		[[nodiscard]] const std::vector<SweepEvent> & pop_next_events();

	public:

		/**
		* \brief Initialize a longitude sweep in the initial state.
		* \param triangles The set of triangles that serve as obstacles.
		* \param longitude The starting longitude of the sweep.
		* \param center The center of
		*/
		LongitudeSweep(const std::vector<Triangle> &triangles,
					   double longitude,
					   const math::Vec3d &center);

        /**
         * \brief Advance the sweep to the next longitude.
         */
		void advance();

		/**
		 * \brief Check whether advance() should be called again.
		 */
		[[nodiscard]] bool has_more_events() const;

		void registerEdge(spherical_geometry::OrderedArcEdge edge);
	};

	inline bool SortByLatitudeAtLongitude::operator()(const spherical_geometry::OrderedArcEdge &a, const spherical_geometry::OrderedArcEdge &b) const {
		return compare_at_longitude(a, b, sweep->current_longitude);
	}
}
#endif //LATITUDE_SWEEP_H
