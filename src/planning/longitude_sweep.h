//
// Created by werner on 28-11-23.
//

#ifndef LATITUDE_SWEEP_H
#define LATITUDE_SWEEP_H

#include <algorithm>
#include <vector>
#include <optional>
#include <queue>
#include <set>
#include <variant>

#include "../math/Vec3.h"

namespace mgodpl
{

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
    * \brief  Compute the longitude of the given point, relative to the given starting longitude.
    * \param starting_longitude  The starting longitude of the sweep.
    * \param longitude           The longitude to compute the relative longitude of.
    * \return                    The relative longitude, as a double in the range [0, 2pi].
    */
    double longitude_ahead_angle(const double starting_longitude, const double longitude);

    /**
    * \brief A triangle in 3D space, with vertices in Cartesian coordinates.
    */
    struct Triangle
    {
        std::array<math::Vec3d, 3> vertices;
    };

    struct Edge
    {
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
    * \brief A range of latitudes defined by two edges at the top and bottom.
    *
    * The range of longitudes covered is the shared range of the two edges.
    *
    * Invariant: the min_latitude_edge has a lower latitude than the max_latitude_edge over the entire shared longitude range.
    */
    struct LatitudeRangeBetweenEdges
    {
        Edge min_latitude_edge;
        Edge max_latitude_edge;
    };

    /**
    * \brief Given an edge, return the latitude of the intersection of the projection of the edge on the sphere with the longitude sweep arc at the given longitude.
    * \param range		    The range of longitudes over which the latitude is valid.
    * \param longitude	    The longitude at which to compute the latitude.
    * \return		        The latitude of the intersection of the projection of the edge on the sphere with the given longitude.
    */
    double latitude(const Edge& edge, double longitude);

    /**
    * \brief	Finds whether, at the last common longitude of the two edges, whether the latitude of b is lower than the latitude of a. (Violating ascending order.)
    * \param a		The first edge.
    * \param b		The second edge.
    * \return		Whether the segments will cross.
    */
    bool edges_will_cross(const Edge& a, const Edge& b);

    /**
    * \brief A struct representing a vertex in a triangle, relative to some center, with the longitude of the vertex.
    */
    struct RelativeVertex
    {
        double longitude = 0.0;
        double latitude = 0.0;
        math::Vec3d local_vertex;
    };

    /**
    * \brief Compute the three relative vertices of a triangle, sorted.
    */
    std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle& triangle, const math::Vec3d& center);

    /**
    * \brief		Compute the intersection of two polar line segments.
    *
    * Implementation based on https://math.stackexchange.com/a/3462227
    *
    * \param a		A unit vector representing the first polar line segment.
    * \param b		The second polar line segment.
    * \return
    */
    math::Vec3d intersection_longitude(const Edge& a, const Edge& b);

    /**
    * \brief A triangle's edges, separated by short edge/long edge.
    */
    struct TriangleEdges
    {
        Edge short_1;
        Edge short_2;
        Edge long_edge;
    };

    /**
    * \brief Construct and label each edge by which role it plays in the triangle during the sweep.
    * \param vertices The vertices of the triangle, sorted by longitude.
    * \return A TriangleEdges struct labeling each edge
    */
    TriangleEdges triangle_edges(const std::array<RelativeVertex, 3>& vertices);

    /**
    * \brief Check whether the middle vertex is above (higher latitude than) the edge between the other two vertices.
    * \param vertices The vertices of the triangle, sorted by longitude.
    * \return True if the middle vertex is at a higher latitude than the edge between the other two vertices.
    */
    bool vertex_is_above_long_edge(const std::array<RelativeVertex, 3>& vertices);

    /**
    * \brief Preprocess a triangle by sorting its vertices by longitude and computing their relative coordinates to the center.
    * \param triangle  The original triangle, in absolute coordinates.
    * \param center    The center of the sphere.
    * \return          The preprocessed triangle.
    */
    std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle& triangle, const math::Vec3d& center);

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
    struct SortByLatitudeAtLongitude
    {
        double longitude;

        bool operator()(const LatitudeRangeBetweenEdges& a, const LatitudeRangeBetweenEdges& b) const;
    };

    /**
    * \brief Compute the signed difference between two longitudes.
    *
    * That is: compute the difference between the two longitudes, and put it into the range [-pi, pi].
    *
    * \return The signed difference between the two longitudes, as a double in the range [-pi, pi].
    */
    double signed_longitude_difference(double first, double second);

    /**
    * \brief A struct representing an event where the sweep arc passes the start of a LatitudeRangeBetweenEdges.
    */
    struct EdgePairStart
    {
        LatitudeRangeBetweenEdges range;
    };

    /**
    * \brief A struct representing an event where the sweep arc passes the end of a LatitudeRangeBetweenEdges.
    */
    struct EdgePairEnd
    {
        LatitudeRangeBetweenEdges range;
    };

    /**
    * \brief A struct representing an event where the sweep arc passes the point where two LatitudeRangeBetweenEdges swap in the order.
    */
    struct EdgePairSwap
    {
        LatitudeRangeBetweenEdges range1, range2;
    };

    /**
    * \brief An event encountered during the longitude sweep.
    */
    struct SweepEvent
    {
        /// The longitude of the event, relative to the starting longitude of the sweep.
        double relative_longitude;

        /// The type of event.
        std::variant<EdgePairStart, EdgePairEnd, EdgePairSwap> event;

        /// Compare two events by their longitude.
        bool operator<(const SweepEvent& other) const
        {
            return relative_longitude < other.relative_longitude;
        }
    };

    /**
    * \brief A struct tracking the state of an ongoing longitude sweep.
    */
    struct LongitudeSweep
    {
        /// The longitude of the last-passed event, or the starting longitude if no events have been processed yet.
        double current_longitude;

        SortByLatitudeAtLongitude comparator;

        /// The ranges of latitudes between edges between `longitude` and the next event (or the end of the sweep).
        /// Warning: this set has a *mutable comparator*; be very careful when changing it.
        std::set<LatitudeRangeBetweenEdges, SortByLatitudeAtLongitude> ranges;

        /// The event queue, sorted by angle ahead of the current longitude. (between 0 and 2pi)
        std::priority_queue<SweepEvent> event_queue;

        /**
        * \brief Initialize a longitude sweep in the initial state.
        * \param triangles The set of triangles that serve as obstacles.
        * \param longitude The starting longitude of the sweep.
        * \param center The center of
        */
        LongitudeSweep(const std::vector<Triangle>& triangles,
                       double longitude,
                       const math::Vec3d& center);

        void process_next_event();

        bool has_more_events() const;
    };
}
#endif //LATITUDE_SWEEP_H
