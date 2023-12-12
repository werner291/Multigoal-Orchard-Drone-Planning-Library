//
// Created by werner on 28-11-23.
//

#include "longitude_sweep.h"

#include <iostream>
#include <random>
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

    std::vector<PaddedOccupiedRangeBetweenEdges> occupied_ranges(
        const std::vector<Triangle>& triangles,
        const math::Vec3d& center,
        double vertical_padding)
    {

        std::vector<PaddedOccupiedRangeBetweenEdges> ranges;
        ranges.reserve(triangles.size());

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

            PaddedOccupiedRangeBetweenEdges rg1{
                .min_latitude_edge = edges.short_1,
                .max_latitude_edge = edges.long_edge,
                .start_padding = padding_from_edge(edges.short_1, vertical_padding),
                .end_padding = padding_from_edge(edges.long_edge, vertical_padding),
                .min_longitude = relative_vertices[0].longitude,
                .max_longitude = relative_vertices[1].longitude
            };

            PaddedOccupiedRangeBetweenEdges rg2{
                .min_latitude_edge = edges.short_2,
                .max_latitude_edge = edges.long_edge,
                .start_padding = padding_from_edge(edges.short_2, vertical_padding),
                .end_padding = padding_from_edge(edges.long_edge, vertical_padding),
                .min_longitude = relative_vertices[1].longitude,
                .max_longitude = relative_vertices[2].longitude
            };

            if (vertex_is_above_long_edge(relative_vertices))
            {
                std::swap(rg1.min_latitude_edge, rg1.max_latitude_edge);
                std::swap(rg2.min_latitude_edge, rg2.max_latitude_edge);

                // Swap the padding too:
                std::swap(rg1.start_padding, rg1.end_padding);
                std::swap(rg2.start_padding, rg2.end_padding);
            }
        }

        return ranges;
    }

    std::set<PaddedOccupiedRangeBetweenEdges, SortByLatitudeAtLongitude> occupied_ranges_crossing_longitude(
        const std::vector<PaddedOccupiedRangeBetweenEdges>& ranges, double longitude)
    {
        std::set<PaddedOccupiedRangeBetweenEdges, SortByLatitudeAtLongitude> result(SortByLatitudeAtLongitude{longitude});

        for (const auto& range : ranges)
        {
            if (signed_longitude_difference(range.min_longitude, longitude) < 0.0 &&
                signed_longitude_difference(range.max_longitude, longitude) > 0.0)
            {
                result.insert(range);
            }
        }

        return result;
    }

    double padding_from_edge(const Edge& edge, double padding)
    {
        double closest_vertex = std::min(edge.vertices[0].norm(), edge.vertices[1].norm());

        return atan(padding / closest_vertex);
    }

    std::set<PaddedOccupiedRangeBetweenEdges, SortByLatitudeAtLongitude> merge_overlapping_ranges(
        const std::set<PaddedOccupiedRangeBetweenEdges, SortByLatitudeAtLongitude>& ranges)
    {
        double longitude = ranges.key_comp().longitude;

        std::set<PaddedOccupiedRangeBetweenEdges, SortByLatitudeAtLongitude> result(ranges.key_comp());

        PaddedOccupiedRangeBetweenEdges current_range = *ranges.begin();
        double current_max_latitude = latitude(current_range.max_latitude_edge, longitude) + current_range.end_padding;

        for (auto it = ++ranges.begin(); it != ranges.end(); ++it)
        {
            const PaddedOccupiedRangeBetweenEdges& next_range = *it;

            double next_min_latitude = latitude(next_range.min_latitude_edge, longitude) - next_range.start_padding;
            double next_max_latitude = latitude(next_range.max_latitude_edge, longitude) + next_range.end_padding;

            if (next_min_latitude > current_max_latitude)
            {
                // The next range is disjoint from the current range.
                result.insert(current_range);

                current_range = next_range;
                current_max_latitude = next_max_latitude;
            }
            else
            {
                // The next range is not disjoint from the current range.
                current_max_latitude = std::max(current_max_latitude, next_max_latitude);
            }
        }

        return result;
    }

    void shuffle_select_edgepairs_in_range(const std::vector<PaddedOccupiedRangeBetweenEdges>& edgepairs,
        std::vector<PaddedOccupiedRangeBetweenEdges> result_buffer, double start_longitude, double end_longitude)
    {
        for (const auto& edgepair : edgepairs)
        {
            if (signed_longitude_difference(edgepair.min_longitude, start_longitude) > 0.0 &&
                signed_longitude_difference(edgepair.max_longitude, end_longitude) < 0.0)
            {
                result_buffer.push_back(edgepair);
            }
        }
    }

    bool vertices_cross_longitude(const std::array<RelativeVertex, 3>& vertices, double longitude)
    {
        return signed_longitude_difference(longitude, vertices[0].longitude) >= 0 &&
            signed_longitude_difference(longitude, vertices[2].longitude) <= 0;
    }

    std::vector<EdgePairStartEnd> range_longitude_start_end_events(const std::vector<PaddedOccupiedRangeBetweenEdges>& ranges)
    {
        std::vector<EdgePairStartEnd> events;

        for (const auto& range : ranges) {
            events.push_back(EdgePairStartEnd{range, START});
            events.push_back(EdgePairStartEnd{range, END});
        }

        std::sort(events.begin(), events.end(), [](const EdgePairStartEnd& a, const EdgePairStartEnd& b) {
            return a.longitude() < b.longitude();
        });

        return events;
    }

    double SortByLatitudeAtLongitude::latitudeAtCurrentLongitude(const PaddedOccupiedRangeBetweenEdges& a) const
    {
        return latitude(a.min_latitude_edge, this->longitude);
    }

    bool SortByLatitudeAtLongitude::operator()(const PaddedOccupiedRangeBetweenEdges& a,
                                               const PaddedOccupiedRangeBetweenEdges& b) const
    {
        return latitudeAtCurrentLongitude(a) - a.start_padding < latitudeAtCurrentLongitude(b) + b.start_padding;
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

    // std::set<FreeRangeBetweenEdges, SortByLatitudeAtLongitude> free_ranges_from_occupied_ranges(
    //     const std::set<OccupiedRangeBetweenEdges, SortByLatitudeAtLongitude>& occupied_ranges)
    // {
    //
    //     double free_start = -M_PI / 2.0;
    //
    //
    //
    // }

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



    LongitudeSweep::LongitudeSweep(const std::vector<Triangle>& triangles, double initial_longitude,
                                   const math::Vec3d& center):
        starting_longitude(initial_longitude),
        current_longitude(initial_longitude)
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

            PaddedOccupiedRangeBetweenEdges rg1{
                .min_latitude_edge = edges.short_1,
                .max_latitude_edge = edges.long_edge
            };

            PaddedOccupiedRangeBetweenEdges rg2{
                .min_latitude_edge = edges.short_2,
                .max_latitude_edge = edges.long_edge
            };

            if (vertex_is_above_long_edge(relative_vertices))
            {
                std::swap(rg1.min_latitude_edge, rg1.max_latitude_edge);
                std::swap(rg2.min_latitude_edge, rg2.max_latitude_edge);
            }

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
    }
    //
    // void LongitudeSweep::process_next_event()
    // {
    //     // Grab the next event:
    //     const SweepEvent event = event_queue.top();
    //     event_queue.pop();
    //
    //     std::cout << "Rel lon: " << event.relative_longitude << " with " << event_queue.size() << " events remaining." << std::endl;
    //
    //     switch (event.event.index())
    //     {
    //     case 0: // EdgePairStart
    //         {
    //             assert(std::get_if<EdgePairStart>(&event.event) != nullptr);
    //             const OccupiedRangeBetweenEdges& range = std::get<EdgePairStart>(event.event).range;
    //
    //             this->current_longitude = event.longitude + DOUBLE_EPSILON;
    //
    //             auto it_already_in = ranges.find(range);
    //
    //             if (it_already_in != ranges.end())
    //             {
    //                 std::cout << "Already in!" << std::endl;
    //
    //                 std::cout << "New range: " << range.min_latitude_edge.vertices[0] << " -> " << range.min_latitude_edge.vertices[1] << " | " << range.max_latitude_edge.vertices[0] << " -> " << range.max_latitude_edge.vertices[1] << std::endl;
    //                 std::cout << "Old range: " << it_already_in->min_latitude_edge.vertices[0] << " -> " << it_already_in->min_latitude_edge.vertices[1] << " | " << it_already_in->max_latitude_edge.vertices[0] << " -> " << it_already_in->max_latitude_edge.vertices[1] << std::endl;
    //             }
    //
    //             // Insert the range into the set.
    //             const auto& [it, inserted] = ranges.insert(range);
    //
    //             assert(inserted);
    //
    //             if (it != ranges.begin())
    //             {
    //                 // Check for potential intersection with the previous range.
    //                 const auto& prev_range = *std::prev(it);
    //
    //                 add_potential_edgecross(prev_range, range);
    //             }
    //
    //             if (std::next(it) != ranges.end())
    //             {
    //                 // Check for potential intersection with the next range.
    //                 const auto& next_range = *std::next(it);
    //
    //                 add_potential_edgecross(range, next_range);
    //             }
    //
    //             // TODO: add/remove intersection events.
    //         }
    //         break;
    //
    //     case 2: // EdgePairEnd
    //         {
    //             assert(std::get_if<EdgePairEnd>(&event.event) != nullptr);
    //             const OccupiedRangeBetweenEdges& range = std::get<EdgePairEnd>(event.event).range;
    //
    //             this->current_longitude = event.longitude - DOUBLE_EPSILON;
    //
    //             // Remove the range from the set.
    //             ranges.erase(range);
    //
    //             // TODO remove intersection events (if applicable? Those should have already triggered by now.) (Or add ones for neighbors that now get close)
    //         }
    //         break;
    //
    //     default:
    //         throw std::runtime_error("Invalid event type");
    //     }
    // }

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

        // Grab the latitude at that point:
        double latitude_1 = latitude(a, end_longitude);
        double latitude_2 = latitude(b, end_longitude);

        bool will_cross = latitude_1 > latitude_2;

        return will_cross;
    }
}
