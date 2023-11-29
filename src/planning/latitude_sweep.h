//
// Created by werner on 28-11-23.
//

#ifndef LATITUDE_SWEEP_H
#define LATITUDE_SWEEP_H

#include <algorithm>
#include <vector>

#include "../math/Vec3.h"

namespace mgodpl
{
    struct Triangle
    {
        std::array<math::Vec3d, 3> vertices;
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

    enum VertexType
    {
        CLOSING,
        OPENING,
        INFLECTION
    };

    /**
     * \brief Compute the longitude of the projection of a point on a sphere (poles on the Z-axis).
     *
     * \param point     The point to compute the longitude of.
     * \param center    The center of the sphere.
     *
     * \return          The longitude, as a double in the range [-pi, pi].
     */
    double longitude(const math::Vec3d& point, const math::Vec3d& center);

    struct PointWithPolar
    {
        math::Vec3d point;
        double latitude, longitude;
    };

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
    };

    /**
     * Given a triangle, find the index of the opening, closing and inflection vertices.
     */
    std::array<size_t, 3> triangle_vertex_types(const Triangle& triangle, const math::Vec3d& center);

    /**
     * \brief Compute a vector of events in order of longitude.
     *
     * For every triangle, three events are created: one for each vertex; Then, all events are sorted by longitude.
     */
    std::vector<VertexEvent> longitude_sweep_events(const std::vector<Triangle>& triangles, const math::Vec3d& center);

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

    struct OngoingIntersections
    {
        std::vector<TriangleIntersection> intersections; // TODO: can we avoid O(n) every deletion? Via tombstones?
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
    OngoingIntersections triangle_intersections(const std::vector<Triangle>& triangles, const math::Vec3d& center,
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
     * Given a set of OngoingIntersections, compute the free latitude ranges.
     *
     * \param   intersections   The ongoing intersections.
     * \param   center          The center of the sphere.
     * \param   sweep_longitude The longitude of the sweep arc.
     * \param   triangles       The triangles that the intersections index into.
     *
     * \return  A vector of latitude ranges, as pairs of doubles in the range [-pi/2, pi/2].
     */
    std::vector<std::array<double, 2>> free_latitude_ranges(const OngoingIntersections& intersections,
                                                            const math::Vec3d& center,
                                                            double sweep_longitude,
                                                            const std::vector<Triangle>& triangles);
}

#endif //LATITUDE_SWEEP_H
