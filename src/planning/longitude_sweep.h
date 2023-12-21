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

namespace mgodpl {
	class LongitudeSweep;

	/**
	* @brief   Compute the latitude of the given point, if projected onto a sphere centered at the given center.
	*
	* @param   point   The point to compute the latitude of.
	* @param   center  The center of the sphere.
	*
	* @return  The latitude, as a double in the range [-pi/2, pi/2].
	*/
	double latitude(const math::Vec3d &point, const math::Vec3d &center);

	/**
	* \brief Compute the longitude of the projection of a point on a sphere (poles on the Z-axis).
	*
	* \param point     The point to compute the longitude of.
	* \param center    The center of the sphere.
	*
	* \return          The longitude, as a double in the range [-pi, pi].
	*/
	double longitude(const math::Vec3d &point, const math::Vec3d &center);

	/**
	* \brief  Compute the longitude of the given point, relative to the given starting longitude.
	* \param starting_longitude  The starting longitude of the sweep.
	* \param longitude           The longitude to compute the relative longitude of.
	* \return                    The relative longitude, as a double in the range [0, 2pi].
	*/
	double longitude_ahead_angle(const double starting_longitude, const double longitude);

	/**
	 * \brief Compute the signed difference between two longitudes.
	 *
	 * That is: compute the difference between the two longitudes, and put it into the range [-pi, pi].
	 *
	 * \return The signed difference between the two longitudes, as a double in the range [-pi, pi].
	 */
	double signed_longitude_difference(double first, double second);

	/**
	* \brief A triangle in 3D space, with vertices in Cartesian coordinates.
	*/
	struct Triangle {
		std::array<math::Vec3d, 3> vertices;

		[[nodiscard]] math::Vec3d normal() const;
	};

	struct Edge {
		std::array<math::Vec3d, 2> vertices;
	};

	/**
	 * \brief A method that computes the angular padding to add to a given polar obstacle point.
	 *
	 * To think about this conceptually, imagine a cylinder of radius r, and a polar point (lat, lon, r).
	 *
 	 * Suppose that the center of one of th bases of the cylinder is at the origin; what are the lat/lon of the median
	 * line of the cylinder, assuming that the polar obstacle point is on the surface of the cylinder?
	 *
	 * Effectively, we have a right-angled triangle with one leg of length r, and the other leg of length arm_radius;
	 * we're looking for the angle between the hypotenuse and the leg of length r.
	 *
	 * That's just atan(arm_radius / r).
	 */
	double angular_padding(double arm_radius, double obstacle_distance);

	/**
	* \brief Given an edge, return the latitude of the intersection of the projection of the edge on the sphere with the longitude sweep arc at the given longitude.
	* \param range		    The range of longitudes over which the latitude is valid.
	* \param longitude	    The longitude at which to compute the latitude.
	* \return		        The latitude of the intersection of the projection of the edge on the sphere with the given longitude.
	*/
	double latitude(const Edge &edge, double longitude);

	/**
	* \brief	Finds whether, at the last common longitude of the two edges, whether the latitude of b is lower than the latitude of a. (Violating ascending order.)
	* \param a		The first edge.
	* \param b		The second edge.
	* \return		Whether the segments will cross.
	*/
	bool edges_will_cross(const Edge &a, const Edge &b);

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
	};

	/**
	* \brief Compute the three relative vertices of a triangle, sorted.
	*/
	std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle &triangle, const math::Vec3d &center);

	struct LongitudeRange {
		double start, end;

		LongitudeRange(double start, double end) : start(start), end(end) {
			assert(signed_longitude_difference(start, end) <= 0);
		}

		[[nodiscard]] bool contains(double longitude) const {
			return signed_longitude_difference(longitude, start) >= 0 && signed_longitude_difference(longitude, end) <= 0;
		}

		[[nodiscard]] bool overlaps(const LongitudeRange &other) const {
			return contains(other.start) || contains(other.end) || other.contains(start) || other.contains(end);
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

		OrderedArcEdge(const RelativeVertex &start, const RelativeVertex &end) : start(start), end(end), id(next_id++) {
			assert(signed_longitude_difference(start.longitude, end.longitude) <= 0);
		}

		double latitudeAtLongitude(double longitude) const {
			// TODO: This is a naive linear interpolation; this should work for now but isn't quite correct.
			assert(signed_longitude_difference(longitude, start.longitude) >= 0);
			assert(signed_longitude_difference(longitude, end.longitude) <= 0);
			double t = (longitude - start.longitude) / (end.longitude - start.longitude); // TODO: This will go wrong when wrapping angles.
			return start.latitude * (1 - t) + end.latitude * t;
		}

		[[nodiscard]] LongitudeRange longitude_range() const {
			return {start.longitude, end.longitude};
		}

		/**
		 * \brief Check whether this edge crosses the given edge.
		 *
		 * That is, the latitude of `other` is below the latitude of this edge at the end of their shared longitude range.
		 *
		 * @pre 	The longitude ranges of the two edges overlap.
		 * @pre     The latitude at `after_longitude` is below `other`.
		 * @pre     The given `after_longitude` is in the longitude range of both edges.
		 *
		 * @param other			  The edge to check for crossing.
		 * @param after_longitude The longitude after which to check for crossing.
		 * @return 		          Whether the edges cross.
		 */
		[[nodiscard]] bool crosses(const OrderedArcEdge &other, double after_longitude) const {
			assert(longitude_range().overlaps(other.longitude_range()));
			assert(longitude_range().contains(after_longitude));

			auto overlap = longitude_range().overlap(other.longitude_range());

			assert(latitudeAtLongitude(after_longitude) <= other.latitudeAtLongitude(after_longitude));

			return latitudeAtLongitude(overlap.end) > other.latitudeAtLongitude(overlap.end);
		}

		[[nodiscard]] RelativeVertex intersection(const OrderedArcEdge &other) const {
			assert(crosses(other, this->longitude_range().overlap(other.longitude_range()).start));

			double m1 = (end.latitude - start.latitude) / (end.longitude - start.longitude);
			double p1 = start.latitude - m1 * start.longitude;

			double m2 = (other.end.latitude - other.start.latitude) / (other.end.longitude - other.start.longitude);
			double p2 = other.start.latitude - m2 * other.start.longitude;

			double longitude = (p2 - p1) / (m1 - m2);
			double latitude = m1 * longitude + p1;

			return {longitude, latitude};
		}

		// Stream output operator:
		friend std::ostream& operator<<(std::ostream& os, const OrderedArcEdge& edge) {
			os << "OrderedArcEdge(id = " << edge.id << ", start=" << edge.start << ", end=" << edge.end << ")";
			return os;
		}
	};

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

	/**
	* \brief Preprocess a triangle by sorting its vertices by longitude and computing their relative coordinates to the center.
	* \param triangle  The original triangle, in absolute coordinates.
	* \param center    The center of the sphere.
	* \return          The preprocessed triangle.
	*/
	std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle &triangle, const math::Vec3d &center);

	/**
	* \brief A struct representing an event where the sweep arc passes the start of a LatitudeRangeBetweenEdges.
	*/
	struct EdgePairStart {
		OrderedArcEdge edge;
	};

	/**
	* \brief A struct representing an event where the sweep arc passes the end of a LatitudeRangeBetweenEdges.
	*/
	struct EdgePairEnd {
		OrderedArcEdge edge;
	};

	/**
	* \brief A struct representing an event where the sweep arc passes the point where two LatitudeRangeBetweenEdges swap in the order.
	*/
	struct EdgePairSwap {
		OrderedArcEdge edge1, edge2;
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

		bool operator()(const OrderedArcEdge &a, const OrderedArcEdge &b) const;

		bool compare_at_longitude(const OrderedArcEdge &a,
								  const OrderedArcEdge &b,
								  double longitude) const;

		bool operator()(const HackyMutable<OrderedArcEdge> &a,
						const HackyMutable<OrderedArcEdge> &b) const {
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
		std::set<HackyMutable<OrderedArcEdge>, SortByLatitudeAtLongitude> ranges;

		/// The event queue, sorted by angle ahead of the current longitude. (between 0 and 2pi)
		/// Using a std::set rather than std::priority_queue to easily detect duplicates and
		/// delete stale events.
		std::set<SweepEvent> event_queue;

		/// Given two edges, check if they cross and, if so, add a EdgePairSwap event to the event queue.
		bool add_potential_edgecross(OrderedArcEdge edge1, OrderedArcEdge edge2);

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
		[[nodiscard]] SweepEvent mkCrossEvent(const OrderedArcEdge &range1, const OrderedArcEdge &range2) const;

		/// Create a SweepEvent for the given edge pair start (does not add it to the queue).
		[[nodiscard]] SweepEvent mkStartEvent(const OrderedArcEdge &rg1) const;

		/// Create a SweepEvent for the given edge pair end (does not add it to the queue).
		[[nodiscard]] SweepEvent mkEndEvent(const OrderedArcEdge &rg1) const;

		/// Remove all events fro the front of the event queue that have the same longitude.
		[[nodiscard]] std::vector<SweepEvent> pop_next_events();

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

		void registerEdge(OrderedArcEdge edge);
	};
}
#endif //LATITUDE_SWEEP_H
