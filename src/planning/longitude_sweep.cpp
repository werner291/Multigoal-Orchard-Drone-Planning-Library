//
// Created by werner on 28-11-23.
//

#include "longitude_sweep.h"

#include <iostream>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

constexpr double DOUBLE_EPSILON = 1e-14; // TODO: Double-check that events aren't this close together.

namespace mgodpl
{
    TriangleEdges triangle_edges(const std::array<RelativeVertex, 3>& vertices)
    {
        return TriangleEdges{
            .short_1 = Edge{vertices[0].local_vertex, vertices[1].local_vertex},
            .short_2 = Edge{vertices[1].local_vertex, vertices[2].local_vertex},
            .long_edge = Edge{vertices[0].local_vertex, vertices[2].local_vertex}
        };
    }

    bool vertex_is_above_long_edge(const std::array<RelativeVertex, 3>& vertices)
    {
        // grab the latitude range at the middle vertex.
        double latitude_edge = latitude(
            Edge{vertices[0].local_vertex, vertices[2].local_vertex},
            longitude(vertices[1].local_vertex, math::Vec3d(0, 0, 0)));

        double latitude_vertex = latitude(vertices[1].local_vertex, math::Vec3d(0, 0, 0));

        return latitude_vertex > latitude_edge;
    }

    std::array<RelativeVertex, 3> sorted_relative_vertices(const Triangle& triangle, const math::Vec3d& center)
    {
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

        std::sort(vertices.begin(), vertices.end(), [](const auto& a, const auto& b)
        {
            return signed_longitude_difference(a.longitude, b.longitude) < 0;
        });

        return vertices;
    }

    bool vertices_cross_longitude(const std::array<RelativeVertex, 3>& vertices, double longitude)
    {
        return signed_longitude_difference(longitude, vertices[0].longitude) >= 0 &&
            signed_longitude_difference(longitude, vertices[2].longitude) <= 0;
    }

    double SortByLatitudeAtLongitude::latitudeAtCurrentLongitude(const LatitudeRangeBetweenEdges& a) const
    {
        return latitude(a.min_latitude_edge, sweep->current_longitude);
    }

    bool SortByLatitudeAtLongitude::operator()(const LatitudeRangeBetweenEdges& a,
                                               const LatitudeRangeBetweenEdges& b) const
    {
        // std::cerr << "Invoked < operator on SortByLatitudeAtLongitude." << std::endl;
        return latitudeAtCurrentLongitude(a) < latitudeAtCurrentLongitude(b);
    }

    double signed_longitude_difference(double first, double second)
    {
        // Compute the difference:
        double difference = first - second;

        // Put it into the range [-pi, pi]
        if (difference > M_PI)
        {
            difference -= 2 * M_PI;
        }
        else if (difference < -M_PI)
        {
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
    bool in_longitude_range(const Edge& edge, double lon)
    {
        return signed_longitude_difference(lon, longitude(edge.vertices[0], math::Vec3d(0, 0, 0))) >= 0 &&
            signed_longitude_difference(lon, longitude(edge.vertices[1], math::Vec3d(0, 0, 0))) <= 0;
    }

    math::Vec3d Edge_intersection(const Edge& a, const Edge& b)
    {
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
        if (dir.dot(a.vertices[0]) < 0)
        {
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
                                                 const LatitudeRangeBetweenEdges range2)
    {
        if (edges_will_cross(range1.min_latitude_edge, range2.min_latitude_edge))
        {
            // If they have a shared enpoint, no event is needed.
            if (range1.min_latitude_edge.vertices[1] == range2.min_latitude_edge.vertices[0])
            {
                assert(false);
                return false;
            }

            // Grab their intersection:
            math::Vec3d intersection = Edge_intersection(range1.min_latitude_edge, range2.min_latitude_edge);

            // Extract lat/lon:
            double intersection_longitude = longitude(intersection, math::Vec3d(0, 0, 0));

            double l1 = latitude(range1.min_latitude_edge, intersection_longitude);
            double l2 = latitude(range2.min_latitude_edge, intersection_longitude);

            assert(std::abs(l1 - l2) <  DOUBLE_EPSILON);

            event_queue.push(SweepEvent{
                longitude_ahead_angle(this->starting_longitude, intersection_longitude),
                .longitude = intersection_longitude,
                EdgePairSwap{range1, range2}
            });

            std::cerr << "Added intersection event between " << range1.min_latitude_edge.vertices[0] << " -> " << range1.min_latitude_edge.vertices[1] << " and " << range2.min_latitude_edge.vertices[0] << " -> " << range2.min_latitude_edge.vertices[1] << std::endl;

            return true;
        }

        return false;

    }

    LongitudeSweep::LongitudeSweep(const std::vector<Triangle>& triangles, double initial_longitude,
                                   const math::Vec3d& center):
        starting_longitude(initial_longitude),
        current_longitude(initial_longitude),
        comparator{this},
        ranges(comparator)
    {
        // Then, iterate over all triangles...
        for (const Triangle& triangle : triangles)
        {

            // If the normal is pointing towards the center, skip this triangle.
            if (triangle.normal().dot(triangle.vertices[0]) < 0)
            {
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

            if (vertex_is_above_long_edge(relative_vertices))
            {
                std::swap(rg1.min_latitude_edge, rg1.max_latitude_edge);
                std::swap(rg2.min_latitude_edge, rg2.max_latitude_edge);
            }

            bool rg1_excluded = false;

            if (on_first_edge)
            {
                // Check to make sure we're on the longitude range:
                assert(in_longitude_range(edges.short_1, current_longitude));
                assert(in_longitude_range(edges.long_edge, current_longitude));
                ranges.insert({rg1});
            }

            bool rg2_excluded = false;

            if (on_second_edge)
            {
                // Check to make sure we're on the longitude range:
                assert(in_longitude_range(edges.short_2, current_longitude));
                assert(in_longitude_range(edges.long_edge, current_longitude));
                ranges.insert({rg2});
            }

            if (!rg1_excluded)
            {
                // Start/end events:
                event_queue.push(SweepEvent{
                    .relative_longitude = longitude_ahead_angle(starting_longitude, relative_vertices[0].longitude),
                    .longitude = relative_vertices[0].longitude,
                    .event = EdgePairStart{rg1}
                });

                event_queue.push(SweepEvent{
                    .relative_longitude = longitude_ahead_angle(starting_longitude, relative_vertices[1].longitude),
                    .longitude = relative_vertices[1].longitude,
                    .event = EdgePairEnd{rg1}
                });
            }
            else
            {
                std::cerr << "Excluded rg1" << std::endl;
            }

            if (!rg2_excluded)
            {
                event_queue.push(SweepEvent{
                    .relative_longitude = longitude_ahead_angle(starting_longitude, relative_vertices[1].longitude),
                    .longitude = relative_vertices[1].longitude,
                    .event = EdgePairStart{rg2}
                });

                event_queue.push(SweepEvent{
                    .relative_longitude = longitude_ahead_angle(starting_longitude, relative_vertices[2].longitude),
                    .longitude = relative_vertices[2].longitude,
                    .event = EdgePairEnd{rg2}
                });
            }
            else
            {
                std::cerr << "Excluded rg2" << std::endl;
            }
        }

        // Then, for all the ongoing latitude ranges, register any potential switching events.
        auto it = ranges.begin();

        do
        {
            const auto range1 = *it;
            ++it;
            if (it == ranges.end())
            {
                break;
            }
            const auto range2 = *it;

            add_potential_edgecross(range1, range2);

        }
        while (it != ranges.end());
    }

    std::vector<SweepEvent> LongitudeSweep::pop_next_events()
    {
        assert(!event_queue.empty());

        std::vector<SweepEvent> events {event_queue.top()};
        event_queue.pop();

        while (!event_queue.empty() && std::abs(event_queue.top().longitude - events[0].longitude) < DOUBLE_EPSILON)
        {
            events.push_back(event_queue.top());
            event_queue.pop();
        }

        return events;
    }

    void LongitudeSweep::delete_affected_ranges(const std::vector<SweepEvent>& events)
    {
        // First pass: delete all affected ranges:
        for (const auto& event : events)
        {
            switch (event.event.index())
            {
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

    void LongitudeSweep::reinsert_nondeleted(const std::vector<SweepEvent>& events)
    {
        // Reinsert all except the deleted ranges.
        for (const auto& event : events)
        {
            switch (event.event.index())
            {
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

    bool LongitudeSweep::check_range_order_validity()
    {
        // Check whether the hacky set construct thingy is still in the right order:
        if (!ranges.empty())
        {
            auto it = ranges.begin();
            auto prev = it;
            ++it;
            while (it != ranges.end())
            {
                if (!ranges.key_comp()(*prev, *it))
                {
                    std::cerr << "Not in order!" << std::endl;
                    std::cerr << "Latitude at current longitude: " << ranges.key_comp().latitudeAtCurrentLongitude(*prev) << " vs "
                        << ranges.key_comp().latitudeAtCurrentLongitude(*it) << std::endl;
                }
                assert(ranges.key_comp()(*prev, *it));

                ++prev;
                ++it;
            }
        }
        return false;
    }

    void LongitudeSweep::process_next_event()
    {

        const std::vector<SweepEvent>& events = pop_next_events();

        std::cerr << "Made it to longitude " << current_longitude << std::endl;

        delete_affected_ranges(events);

        check_range_order_validity();

        // Then, push the current longitude a bit ahead of the event longitude.
        this->current_longitude = events[0].longitude + DOUBLE_EPSILON;

        check_range_order_validity();

        reinsert_nondeleted(events);

        // Finally, check if the new neighbors need any crossing events:
        for (const SweepEvent evt: events)
        {
            switch (evt.event.index())
            {
            case 0: // Range start.
                {
                    const LatitudeRangeBetweenEdges& range = std::get<EdgePairStart>(evt.event).range;

                    // Find the range.
                    auto it = ranges.find({range});

                    if (it != ranges.begin())
                    {
                        add_potential_edgecross(*std::prev(it), *it);
                    }

                    if (std::next(it) != ranges.end())
                    {
                        add_potential_edgecross(*it, *std::next(it));
                    }
                }
                break;

            case 1: // Range deletion.
                {
                    // Find where it would be inserted.
                    const LatitudeRangeBetweenEdges& range = std::get<EdgePairEnd>(evt.event).range;

                    auto lower = ranges.lower_bound(range); // TODO: Double-check if these are the right ones.
                    auto upper = ranges.upper_bound(range);
                    if (lower != ranges.end() && upper != ranges.end())
                    {
                        add_potential_edgecross(*lower, *upper);
                    }
                }
                break;
            case 2: // Swapping; we treat this as just two insertions. Slightly inefficient, but it works fine... right?
                {
                    const LatitudeRangeBetweenEdges& range1 = std::get<EdgePairSwap>(evt.event).range1;
                    const LatitudeRangeBetweenEdges& range2 = std::get<EdgePairSwap>(evt.event).range2;

                    for (const LatitudeRangeBetweenEdges& range : {range1, range2})
                    {
                        // Find the range.
                        auto it = ranges.find({range});

                        if (it != ranges.begin())
                        {
                            add_potential_edgecross(*std::prev(it), *it);
                        }

                        if (std::next(it) != ranges.end())
                        {
                            add_potential_edgecross(*it, *std::next(it));
                        }
                    }
                }
                break;
            default: throw std::runtime_error("Invalid event type");
            }
        }

        check_range_order_validity();

    }

    bool LongitudeSweep::has_more_events() const
    {
        return !event_queue.empty();
    }

    double longitude_ahead_angle(const double starting_longitude, const double longitude)
    {
        double a_longitude = signed_longitude_difference(longitude, starting_longitude);
        if (a_longitude < 0) a_longitude += 2 * M_PI;
        return a_longitude;
    }

    math::Vec3d Triangle::normal() const
    {
        return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
    }

    double latitude(const math::Vec3d& point, const math::Vec3d& center)
    {
        math::Vec3d delta = point - center;

        // Distance from the vertical axis through the center of the sphere.
        const double distance_xy = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

        // Compute the latitude.
        return std::atan2(delta.z(), distance_xy);
    }

    double longitude(const math::Vec3d& point, const math::Vec3d& center)
    {
        math::Vec3d delta = point - center;

        return std::atan2(delta.y(), delta.x());
    }

    double angular_padding(double arm_radius, double obstacle_distance)
    {
        return atan(arm_radius / obstacle_distance);
    }

    double latitude(const Edge& edge, double at_longitude)
    {
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
        if (direction.dot(edge.vertices[0]) < 0)
        {
            assert(direction.dot(edge.vertices[1]) < 0);
            direction = -direction;
        }

        return latitude(direction, math::Vec3d(0, 0, 0));
    }

    bool edges_will_cross(const Edge& a, const Edge& b)
    {
        double end_longitude_1 = longitude(a.vertices[1], math::Vec3d(0, 0, 0));
        double end_longitude_2 = longitude(b.vertices[1], math::Vec3d(0, 0, 0));

        double end_longitude = signed_longitude_difference(end_longitude_1, end_longitude_2) < 0
                                   ? end_longitude_1
                                   : end_longitude_2;

        // Sanity check: make sure that the end longitude is in the longitude range of both edges:
        assert(in_longitude_range(a, end_longitude));
        assert(in_longitude_range(b, end_longitude));

        // Check to make sure the end longitudes are different:
        if (std::abs(signed_longitude_difference(end_longitude_1, end_longitude_2)) < DOUBLE_EPSILON)
        {
            return false;
        }

        // Grab the latitude at that point:
        double latitude_1 = latitude(a, end_longitude);
        double latitude_2 = latitude(b, end_longitude);

        bool will_cross = latitude_1 > latitude_2;

        return will_cross;
    }
}
