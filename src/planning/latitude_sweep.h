//
// Created by werner on 28-11-23.
//

#ifndef LATITUDE_SWEEP_H
#define LATITUDE_SWEEP_H

#include <vector>
#include <optional>

#include "../math/Vec3.h"

namespace mgodpl
{
	/**
	 * \brief A triangle in 3D space, with vertices in Cartesian coordinates.
	 */
    struct Triangle
    {
        std::array<math::Vec3d, 3> vertices;
    };

	/**
	 * A struct that describes whether a latitude is the start or end of a range.
	 *
	 * For a start, the latitude is the lower bound of the range.
	 * For an end, the latitude is the upper bound of the range.
	 */
	enum LatitudeStartEnd {
		START,
		END
	};

	/**
	 * \brief A struct representing a latitude/longitude pair, with a radius.
	 */
    struct PolarCoordinates
    {
        double latitude;
        double longitude;
		double radius;
    };

	/**
	 * \brief A struct representing an ongoing intersection of a sweep arc with an edge of a spherical triangle.
	 *
	 * Invariant: start.longitude <= end.longitude (taking wrapping into account), and start.latitude <= end.latitude.
	 */
	struct OngoingEdgeIntersection {
		PolarCoordinates start;
		PolarCoordinates end;
		LatitudeStartEnd start_end;

		// sorting operator by starting latitude.
	};

    /**
     * \brief A triangle whose vertices are in polar coordinates, sorted by longitude (taking wrapping into account).
     */
    struct OrderedPolarTriangle
    {
        std::array<PolarCoordinates, 3> vertices;
    };

    /**
     * @brief   Compute the latitude of the given point, if projected onto a sphere centered at the given center.
     *
     * @param   point   The point to compute the latitude of.
     * @param   center  The center of the sphere.
     *
     * @return  The latitude, as a double in the range [-pi/2, pi/2].
     */
    double latitude(const math::Vec3d& point, const math::Vec3d& center);

    /**
     * \brief Compute the longitude of the projection of a point on a sphere (poles on the Z-axis).
     *
     * \param point     The point to compute the longitude of.
     * \param center    The center of the sphere.
     *
     * \return          The longitude, as a double in the range [-pi, pi].
     */
    double longitude(const math::Vec3d& point, const math::Vec3d& center);

    /**
     * \brief   Convert a point to polar coordinates.
     * \param point The point to convert.
     * \param center The center of the sphere.
     * \return  The polar coordinates of the point.
     */
    PolarCoordinates polar_coordinates(const math::Vec3d& point, const math::Vec3d& center);

    /**
     * \brief A struct representing the type of an event in the sweep algorithm.
     */
    enum EventType
    {
        /// The sweep arc starts intersecting the triangle, encountering the opening vertex.
        TRIANGLE_INTERSECTION_START,
        /// The sweep arc hits a vertex without entering or leaving the triangle.
        INTERNAL_TRIANGLE_VERTEX,
        /// The sweep arc hits a vertex and enters the triangle.
        TRIANGLE_INTERSECTION_END,
    };

    /**
     * \brief A struct representing an event in the sweep algorithm.
     */
    struct VertexEvent
    {
        /// The type of the event.
        EventType type;
        /// The triangle index of the triangle involved in the event.
        size_t triangle_index;
        /// The index of the vertex within the triangle involved in the event.
        size_t vertex_index;
        /// The longitude of the vertex involved in the event.
        double longitude;
		/// The longitude of the vertex reletive to the start of the sweep.
		double relative_longitude;
    };

    /**
     * Given a triangle, find the index of the opening, closing and inflection vertices.
     */
    std::array<size_t, 3> triangle_vertices_by_longitude(const Triangle& triangle, const math::Vec3d& center);

    /**
     * \brief Compute a vector of events in order of longitude.
     *
     * For every triangle, three events are created: one for each vertex; Then, all events are sorted by longitude.
     */
	std::vector<VertexEvent> longitude_sweep_events(const std::vector<Triangle> &triangles,
													const math::Vec3d &center,
													const double starting_longitude);

    /**
     * \brief Compute the signed difference between two longitudes.
     *
     * That is: compute the difference between the two longitudes, and put it into the range [-pi, pi].
     *
     * \return The signed difference between the two longitudes, as a double in the range [-pi, pi].
     */
    double signed_longitude_difference(double first, double second);

    /**
     * Given a segment on the sphere, defined by two points, and a pole-to-pole latitude line,
     * compute the latitude of the intersection point.
     *
     * \param segment_start     The start of the segment (to be projected on the sphere)
     * \param segment_end       The end of the segment (to be projected on the sphere)
     * \param sweep_longitude   The longitude of the sweep line.
     * \param center            The center of the sphere.
     *
     * \pre                     The intersection exists (checked by assertion), and the segment points are in the correct order.
     *
     * \return                 The latitude of the intersection point in the range [-pi/2, pi/2].
     */
	double segment_intersection_latitude(
			const math::Vec3d& segment_start,
			const math::Vec3d& segment_end,
			double sweep_longitude,
			const math::Vec3d& center);

	PolarCoordinates segment_intersection(
			const math::Vec3d& segment_start,
			const math::Vec3d& segment_end,
			double sweep_longitude,
			const math::Vec3d& center);

    /**
     * \brief A struct describing the (ongoing) intersection of a sweep arc with an edge of a spherical triangle.
     *
     * Can be used together with segment_intersection_latitude to compute the latitude of the intersection.
     */
    struct ArcSegmentIntersection
    {
        size_t triangle_index;
        size_t edge_start_vertex_index;
        size_t edge_end_vertex_index;
    };

    /**
     * \brief A struct describing the (ongoing) intersection of a sweep arc with a triangle.
     *
     * This struct describes the intersection of a sweep arc with a triangle, as the sweep arc is moving from left to right.
     *
     * Effectively, we can use it to define a function from longitude (over the intersection range) to a range of latitudes.
     */
    struct TriangleIntersection
    {
        size_t triangle_index;
        size_t opening_vertex_index;
        size_t closing_vertex_index;
        size_t inflection_vertex_index;
        double opening_longitude;
        double closing_longitude;
        double inflection_longitude;
    };

    /**
     * \brief Given a TriangleIntersection and a longitude, return the two ongoing segment intersections.
     *
     * Specifically, if the longitude is less than the inflection longitude,
     * the intersections are with the edges (opening, inflection), (opening, closing).
     * Otherwise they are with the edges (opening, closing), (inflection, closing).
     *
     * \pre     The longitude is in the range [opening_longitude, closing_longitude]. (Checked by assertion.)
     *
     * \returns An array of two ArcSegmentIntersections, describing the two ongoing segment intersections.
     */
    std::array<ArcSegmentIntersection, 2> current_segment_intersections(const TriangleIntersection& intersection,
                                                                        double longitude);

    /**
     * \brief Given an ongoing triangle intersection and a longitude, compute the latitude range of the intersection.
     *
     * \param   intersection    The triangle intersection.
     * \param   longitude       The longitude of the sweep arc.
     * \param   center          The center of the sphere.
     * \param   triangles       The vector of triangles that the intersection indexes into.
     *
     * \pre     The longitude is in the range [opening_longitude, closing_longitude]. (Checked by assertion.)
     *
     * \return A range of latitudes, as a pair of doubles in the range [-pi/2, pi/2] (note: bounds are unordered).
     */
    std::array<double, 2> latitude_range(
        const TriangleIntersection& intersection,
        double longitude,
        const math::Vec3d& center,
        const std::vector<Triangle>& triangles
    );

	/**
	 * \brief	Given an ongoing triangle intersection and a longitude, compute the endpoints of the intersection arc.
	 *
	 * @param intersection 		The triangle intersection.
	 * @param longitude 		The longitude of the sweep arc.
	 * @param center 			The center of the sphere.
	 * @param triangles 		The vector of triangles that the intersection indexes into.
	 *
	 * \pre 	The longitude is in the range [opening_longitude, closing_longitude]. (Checked by assertion.)
	 *
	 * @return 					The endpoints of the intersection arc, as a pair of polar coordinates.
	 */
	std::array<PolarCoordinates, 2> intersection_arc(const TriangleIntersection& intersection,
													 double longitude,
													 const math::Vec3d& center,
													 const std::vector<Triangle>& triangles);

	/**
     * \brief Given an ongoing triangle intersection and a longitude, compute the latitude range of the intersection,
     * 		  but restrict the range such that a minimum linear padding is added to the range.
     *
     * We'd like to find the range of latitudes, such that for every latitude `lat` in the range,
     * we can construct a rectangle whose median line is a ray originating at the center with polar
     * direction (lat,lon), with width `2 * vertical_padding`.
     *
     * TODO: This is vague; need to write down formally with some pictures.
     *
     * \param   intersection      The triangle intersection.
     * \param   longitude         The longitude of the sweep arc.
     * \param   vertical_padding  The amount of padding to add to the vertical range.
     * \param   center            The center of the sphere.
     * \param   triangles         The vector of triangles that the intersection indexes into.
     *
     * \pre     The longitude is in the range [opening_longitude, closing_longitude]. (Checked by assertion.)
     *
     * \return A range of latitudes, as a pair of doubles in the range [-pi/2, pi/2] (note: bounds are unordered).
     */
	std::array<double, 2> vertical_padded_latitude_range(
			const TriangleIntersection& intersection,
			double longitude,
			double vertical_padding,
			const math::Vec3d& center,
			const std::vector<Triangle>& triangles
	);

    struct OngoingIntersections
    {
        std::vector<std::optional<TriangleIntersection>> intersections;
    };

    /**
     * \brief Given a vector of triangles, compute the intersections of the sweep arc with the triangles.
     *
     * TODO: This runs in O(n) time; that said, it's in the initialization of the algorithm,
     * so it should not increase the asymptotic complexity of the algorithm overall. Still,
     * it's possibly a big constant, so can we do better?
     *
     * \param triangles         The triangles to compute the intersections with.
     * \param center            The center of the sphere.
     * \param sweep_longitude   The longitude of the sweep arc.
     *
     * \return A vector of TriangleIntersections, in no particular order.
     */
    OngoingIntersections triangle_intersections(const std::vector<Triangle>& triangles,
                                                const math::Vec3d& center,
                                                double sweep_longitude);

    /**
     * Update the ongoing intersections with a given event.
     *
     * \param   intersections   The ongoing intersections. (Will be modified.)
     * \param   event           The event to update the intersections with.
     */
    void update_intersections(OngoingIntersections& intersections, const VertexEvent& event,
                              const std::vector<Triangle>& triangles, const math::Vec3d& center);

	/**
	 * \brief 	Run the sweepline algorithm, accumulating a vector of monotone ranges.
	 *
	 * Specifically, given a vector of triangles, a center, and a starting longitude,
	 * will return a representation of a function from longitude to a vector of triangles
	 * that are intersected by the sweep arc at that longitude.
	 *
	 * \param 	triangles 			The triangles to compute the intersections with.
	 * \param 	center 				The center of the sphere.
	 * \param 	starting_longitude 	The initial longitude of the sweep arc (default 0.0), in range [-pi, pi].
	 */
	std::vector<std::pair<double, OngoingIntersections>> run_sweepline(const std::vector<Triangle>& triangles,
																	   const math::Vec3d& target,
																	   double starting_longitude = 0.0);

	/**
	 * @brief Compute the positive angle between two longitudes.
	 *
	 * That is:  how much must the longitude angle increase to go from starting_longitude to longitude, possibly wrapping around?
	 *
	 * @param starting_longitude
	 * @param longitude
	 * @return
	 */
	double longitude_ahead_angle(const double starting_longitude, const double longitude);

    /**
     * Given a set of OngoingIntersections, compute the free latitude ranges.
     *
     * \param   intersections   The ongoing intersections.
     * \param   center          The center of the sphere.
     * \param   sweep_longitude The longitude of the sweep arc.
     * \param   triangles       The triangles that the intersections index into.
     *
     * \return  A vector of latitude ranges, as pairs of doubles in the range [-pi/2, pi/2].
     */
	std::vector<std::array<double, 2>> free_latitude_ranges(const OngoingIntersections &intersections,
															const math::Vec3d &center,
															double sweep_longitude,
															const std::vector<Triangle> &triangles,
															double vertical_padding);

    /**
     * \brief   Convert a triangle to longitude-sorted polar coordinates.
     *
     * \warning The vertices will be reordered.
     *
     * \param triangle The triangle to convert.
     * \param center The center of the sphere.
     */
    OrderedPolarTriangle polar_triangle(const Triangle& triangle, const math::Vec3d& center);

	/**
	 * An encapsulation of the longitude sweep operation, tracking
	 * the state of the sweep and related information across the operation.
	 *
	 * The "events" are longitudes at which something discrete happens,
	 * such as the sweep arc intersecting the starting or ending vertex
	 * of a triangle.
	 *
	 * In between "events" (or before/after the first/last event), we get "longitude ranges",
	 * which are the ranges of longitudes for which the sweep arc is intersecting
	 * the same set of triangles.
	 */
	class LongitudeSweep {

		/// A vector of triangles that serve as our obstacles.
		const std::vector<Triangle> &triangles;
		/// All intersections of the current sweep arc with the projections of the triangles.
		OngoingIntersections intersections;
		/// A vector of all VertexEvents, sorted by longitude, relative too the starting longitude.
		std::vector<VertexEvent> events;
		/// A counter for how many events have been passed (not counting passing the starting longitude).
		size_t events_passed;
		/// The center of the sphere being swept.
		math::Vec3d target;
		/// The initial longitude of the sweep arc at the start of the sweep.
		double starting_longitude;

	public:

		/**
		 * @brief Construct a new LongitudeSweep, in the initial state.
		 *
		 * @param triangles 				The triangles to compute the intersections with.
		 * @param target 					The center of the swept sphere.
		 * @param starting_longitude 		The initial longitude of the sweep arc (default 0.0), in range [-pi, pi].
		 */
		LongitudeSweep(const std::vector<Triangle> &triangles, const math::Vec3d &target, double starting_longitude = 0.0);

		/**
		 * Return the number of ranges. (That is, the number of events + 1.)
		 */
		[[nodiscard]] size_t number_of_ranges() const;

		[[nodiscard]] size_t ranges_passed() const;

		[[nodiscard]] bool has_more_ranges() const;

		/**
		 * Advance the sweep arc until it passes the longitude of the next event,
		 * changing the current longitude range.
		 *
		 * \pre has_more_events() == true (checked by assertion).
		 */
		void advance();

		/**
		 * @brief Get the longitude of the previous and the next event that the sweep arc will pass.
		 *
		 * That is, if the arc has to pass the first event, then this is [starting_longitude, first_event.longitude].
		 * If it is in between two events, then this is [previous_event.longitude, next_event.longitude].
		 * If it is past the last event, then this is [last_event.longitude, starting_longitude + 2 * pi].
		 *
		 * @return A pair of doubles, representing the longitude range.
		 */
		[[nodiscard]] std::array<double, 2> current_longitude_range();

		/**
		 * @brief Return the set of ongoing intersections during the current longitude range.
		 */
		[[nodiscard]] const OngoingIntersections& current_intersections() const;
	};

}

#endif //LATITUDE_SWEEP_H
