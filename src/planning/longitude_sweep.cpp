//
// Created by werner on 28-11-23.
//

#include "longitude_sweep.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

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
        std::array<RelativeVertex, 3> vertices {
            RelativeVertex{longitude(triangle.vertices[0], center),latitude(triangle.vertices[0], center), triangle.vertices[0] - center},
            RelativeVertex{longitude(triangle.vertices[1], center), latitude(triangle.vertices[1], center), triangle.vertices[1] - center},
            RelativeVertex{longitude(triangle.vertices[2], center), latitude(triangle.vertices[2], center), triangle.vertices[2] - center}
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

    bool SortByLatitudeAtLongitude::operator()(const LatitudeRangeBetweenEdges& a,
        const LatitudeRangeBetweenEdges& b) const
    {
        double l1 = latitude(a.min_latitude_edge, longitude);
        double l2 = latitude(b.min_latitude_edge, longitude);

        return l1 < l2;
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

    math::Vec3d intersection_longitude(const Edge& a, const Edge& b)
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

        // The direction vector of the intersection line is the cross product of the two plane normals.
        math::Vec3d direction = nA.cross(nB).normalized();

        // Get the longitude of the direction vector.
        double intersection_longitude = longitude(direction, math::Vec3d(0, 0, 0));

        // Sanity check: make sure it's in the longitude ranges of both segments.
        assert(signed_longitude_difference(intersection_longitude, longitude(a.vertices[0], math::Vec3d(0, 0, 0))) >= 0);
        assert(signed_longitude_difference(intersection_longitude, longitude(a.vertices[1], math::Vec3d(0, 0, 0))) <= 0);

        assert(signed_longitude_difference(intersection_longitude, longitude(b.vertices[0], math::Vec3d(0, 0, 0))) >= 0);
        assert(signed_longitude_difference(intersection_longitude, longitude(b.vertices[1], math::Vec3d(0, 0, 0))) <= 0);

        // Return the longitude.
        return direction;
    }

    LongitudeSweep::LongitudeSweep(const std::vector<Triangle>& triangles, double initial_longitude, const math::Vec3d& center):
        current_longitude(initial_longitude),
        comparator {initial_longitude},
        ranges(comparator),
    event_queue()
    {

        // Then, iterate over all triangles...
        for (const Triangle& triangle : triangles)
        {
            std::array<RelativeVertex, 3> relative_vertices = sorted_relative_vertices(triangle, center);

            TriangleEdges edges = triangle_edges(relative_vertices);

            std::array<bool, 3> vertex_passed = {
                signed_longitude_difference(current_longitude, relative_vertices[0].longitude) >= 0,
                signed_longitude_difference(current_longitude, relative_vertices[1].longitude) >= 0,
                signed_longitude_difference(current_longitude, relative_vertices[2].longitude) >= 0
            };

            bool on_first_edge = vertex_passed[0] && vertex_passed[1];
            bool on_second_edge = vertex_passed[1] && vertex_passed[2];
            bool on_long_edge = vertex_passed[0] && vertex_passed[2];

            // Sanity check: on_long_edge implies on_first_edge and on_second_edge.
            assert(!on_long_edge || (on_first_edge && on_second_edge));

            LatitudeRangeBetweenEdges rg1 {
                .min_latitude_edge = edges.short_1,
                .max_latitude_edge = edges.long_edge
            };

            LatitudeRangeBetweenEdges rg2 {
                .min_latitude_edge = edges.short_2,
                .max_latitude_edge = edges.long_edge
            };

            if (vertex_is_above_long_edge(relative_vertices))
            {
                std::swap(rg1.min_latitude_edge, rg1.max_latitude_edge);
                std::swap(rg2.min_latitude_edge, rg2.max_latitude_edge);
            }

            if (on_first_edge)
            {
                ranges.insert(rg1);
            }

            if (on_second_edge)
            {
                ranges.insert(rg2);
            }

            // Start/end events:
            event_queue.push(SweepEvent{
                .relative_longitude = longitude_ahead_angle(current_longitude, relative_vertices[0].longitude),
                .event = EdgePairStart{rg1}
            });

            event_queue.push(SweepEvent{
                .relative_longitude = longitude_ahead_angle(current_longitude, relative_vertices[2].longitude),
                .event = EdgePairStart{rg2}
            });
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

            if (edges_will_cross(range1.min_latitude_edge, range2.min_latitude_edge))
            {
                // Grab their intersection:
                math::Vec3d intersection = intersection_longitude(range1.min_latitude_edge, range2.min_latitude_edge);

                // Extract lat/lon:
                double intersection_latitude = latitude(intersection, math::Vec3d(0, 0, 0));
                double intersection_longitude = longitude(intersection, math::Vec3d(0, 0, 0));

                event_queue.push(SweepEvent {
                    longitude_ahead_angle(current_longitude, intersection_longitude),
                    EdgePairSwap{range1, range2}
                });
            }
        } while (it != ranges.end());

    }

    void LongitudeSweep::process_next_event()
    {
        // Grab the next event:
        const SweepEvent event = event_queue.top();
        event_queue.pop();

        switch (event.event.index())
        {
        case 0: // EdgePairStart
            {
                const LatitudeRangeBetweenEdges& range = std::get<EdgePairStart>(event.event).range;

                this->comparator.longitude = event.relative_longitude;

                // Insert the range into the set.
                ranges.insert(range);
            }
            break;
        case 1: // EdgePairEnd
            {
                const LatitudeRangeBetweenEdges& range = std::get<EdgePairEnd>(event.event).range;

                this->comparator.longitude = event.relative_longitude;

                // Remove the range from the set.
                ranges.erase(range);
            }
            break;
        case 2: // EdgePairSwap
            {
                const LatitudeRangeBetweenEdges& range1 = std::get<EdgePairSwap>(event.event).range1;
                const LatitudeRangeBetweenEdges& range2 = std::get<EdgePairSwap>(event.event).range2;

                // Wow this is ugly... TODO Does that const value work properly? Do we need to do a dynamic epsilon?
                this->comparator.longitude = event.relative_longitude - 1e-20;

                // Delete them from the set.
                ranges.erase(range1);
                ranges.erase(range2);

                this->comparator.longitude = event.relative_longitude + 1e-20;

                ranges.insert(range1);
                ranges.insert(range2);

                this->comparator.longitude = event.relative_longitude;
            }
            break;

        default:
            throw std::runtime_error("Invalid event type");
        }
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
            cos(at_longitude),
            sin(at_longitude),
            0
        );

        // Now it's easy: get the direction of the intersection line.
        math::Vec3d direction = edge_normal.cross(lon_direction);

        // Sanity check: make sure the longitude is the one we expect.
        assert(std::abs(longitude(direction, math::Vec3d(0, 0, 0)) - at_longitude) < 1e-6);

        return latitude(direction, math::Vec3d(0, 0, 0));
    }

    bool edges_will_cross(const Edge& a, const Edge& b)
    {
        double end_longitude_1 = longitude(a.vertices[1], math::Vec3d(0, 0, 0));
        double end_longitude_2 = longitude(b.vertices[1], math::Vec3d(0, 0, 0));

        double end_longitude = signed_longitude_difference(end_longitude_1, end_longitude_2) < 0 ?
                                   end_longitude_1 : end_longitude_2;

        // Grab the latitude at that point:
        double latitude_1 = latitude(a, end_longitude);
        double latitude_2 = latitude(b, end_longitude);

        bool will_cross = latitude_1 > latitude_2;

        return will_cross;
    }
}
