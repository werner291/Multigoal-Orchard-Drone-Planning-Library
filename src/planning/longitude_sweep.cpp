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
                rg1_excluded = !ranges.insert({rg1}).second;
            }

            bool rg2_excluded = false;

            if (on_second_edge)
            {
                // Check to make sure we're on the longitude range:
                assert(in_longitude_range(edges.short_2, current_longitude));
                assert(in_longitude_range(edges.long_edge, current_longitude));
                rg2_excluded = !ranges.insert({rg2}).second;
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

            add_potential_edgecross(range1.interior, range2.interior);


        }
        while (it != ranges.end());
    }

    void LongitudeSweep::process_next_event()
    {
        // Grab the next event:
        const SweepEvent event = event_queue.top();
        event_queue.pop();

        // If the current event is a swap, pop any duplicates:
        // Duplicate events are totally normal. For instance, consider the following orderings:
        //
        // ABCDE
        // (Swap C and D)
        // ABDCE
        // (Swap D and B)
        // ADBCE
        //
        // Notice that B and C end up together again; if we only look at neighboring ranges, we'll end up with a duplicate events/
        while (!event_queue.empty() && event_queue.top().longitude == event.longitude && event.event.index() == 1 && event_queue.top().event.index() == 1)
        {
            // Make sure the ranges are in the set of ranges:
            assert(ranges.find({std::get<EdgePairSwap>(event.event).range1}) != ranges.end());
            assert(ranges.find({std::get<EdgePairSwap>(event.event).range2}) != ranges.end());

            assert(ranges.find({std::get<EdgePairSwap>(event_queue.top().event).range1}) != ranges.end());
            assert(ranges.find({std::get<EdgePairSwap>(event_queue.top().event).range2}) != ranges.end());

            std::cerr << "Found potential duplicate event with same longitude: " << event_queue.top().longitude << std::endl;

            std::cerr << "Current range1: " << std::get<EdgePairSwap>(event.event).range1.min_latitude_edge.vertices[0] << " -> " << std::get<EdgePairSwap>(event.event).range1.min_latitude_edge.vertices[1] << " | " << std::get<EdgePairSwap>(event.event).range1.max_latitude_edge.vertices[0] << " -> " << std::get<EdgePairSwap>(event.event).range1.max_latitude_edge.vertices[1] << std::endl;
            std::cerr << "Potential duplicate: " << std::get<EdgePairSwap>(event_queue.top().event).range1.min_latitude_edge.vertices[0] << " -> " << std::get<EdgePairSwap>(event_queue.top().event).range1.min_latitude_edge.vertices[1] << " | " << std::get<EdgePairSwap>(event_queue.top().event).range1.max_latitude_edge.vertices[0] << " -> " << std::get<EdgePairSwap>(event_queue.top().event).range1.max_latitude_edge.vertices[1] << std::endl;

            // Make sure these are the same ranges.
            assert(std::get<EdgePairSwap>(event_queue.top().event).range1 == std::get<EdgePairSwap>(event.event).range1);
            assert(std::get<EdgePairSwap>(event_queue.top().event).range2 == std::get<EdgePairSwap>(event.event).range2);

            std::cerr << "Popping duplicate event!" << std::endl;
            event_queue.pop();
        }

        switch (event.event.index())
        {
        case 0: // EdgePairStart
            {
                assert(std::get_if<EdgePairStart>(&event.event) != nullptr);
                const LatitudeRangeBetweenEdges& range = std::get<EdgePairStart>(event.event).range;

                // Put the current longitude a bit ahead of the event longitude.
                // This is to prevent numerical issues when comparing edges that start in the same place.
                this->current_longitude = event.longitude + DOUBLE_EPSILON;

                // Insert the range into the set.
                const auto& [it, inserted] = ranges.insert({range});

                if (!inserted)
                {
                    std::cerr << "Not inserted!" << std::endl;

                    std::cerr << "New range: " << range.min_latitude_edge.vertices[0] << " -> " << range.min_latitude_edge.vertices[1] << " | " << range.max_latitude_edge.vertices[0] << " -> " << range.max_latitude_edge.vertices[1] << std::endl;
                    std::cerr << "Old range: " << it->interior.min_latitude_edge.vertices[0] << " -> " << it->interior.min_latitude_edge.vertices[1] << " | " << it->interior.max_latitude_edge.vertices[0] << " -> " << it->interior.max_latitude_edge.vertices[1] << std::endl;
                }
                else
                {
                    if (it != ranges.begin())
                    {
                        // Check for potential intersection with the previous range.
                        const auto& prev_range = std::prev(it)->interior;

                        add_potential_edgecross(prev_range, range);
                    }

                    if (std::next(it) != ranges.end())
                    {
                        // Check for potential intersection with the next range.
                        const auto& next_range = std::next(it)->interior;

                        add_potential_edgecross(range, next_range);
                    }
                }

                std::cerr << "Inserted range: " << range.min_latitude_edge.vertices[0] << " -> " << range.min_latitude_edge.vertices[1] << " | " << range.max_latitude_edge.vertices[0] << " -> " << range.max_latitude_edge.vertices[1] << std::endl;
            }
            break;
        case 1: // EdgePairSwap
            {

                assert(event_queue.top().longitude > event.longitude + DOUBLE_EPSILON);

                assert(std::get_if<EdgePairSwap>(&event.event) != nullptr);
                const LatitudeRangeBetweenEdges& range1 = std::get<EdgePairSwap>(event.event).range1;
                const LatitudeRangeBetweenEdges& range2 = std::get<EdgePairSwap>(event.event).range2;

                auto it1 = ranges.find({range1});
                auto it2 = ranges.find({range2});
                assert(std::next(it1) == it2);

                // Double-check that we actually found the right values:
                assert(it1->interior == range1);
                assert(it2->interior == range2);

                {
                    double l1 = latitude(range1.min_latitude_edge, event.longitude - 0.1);
                    double l2 = latitude(range2.min_latitude_edge, event.longitude - 0.1);
                    assert(l1 < l2);
                }
                {
                    double l1 = latitude(range1.min_latitude_edge, event.longitude + 0.1);
                    double l2 = latitude(range2.min_latitude_edge, event.longitude + 0.1);
                    assert(l1 > l2);
                }

                // Swap the ranges (This point is the entire reason why we have the HackyMutable struct)
                // Note: We're not officially allowed to do this. TODO: Find a non-hacky alternative, maybe? The order *should* be preserved...
                std::swap(
                    it1->interior, it2->interior
                );
                std::cerr << "Swapped ranges:" << std::endl;
                std::cerr << "Next range: " << it2->interior.min_latitude_edge.vertices[0] << " -> " << it2->interior.min_latitude_edge.vertices[1] << " | " << it2->interior.max_latitude_edge.vertices[0] << " -> " << it2->interior.max_latitude_edge.vertices[1] << std::endl;
                std::cerr << "Prev range: " << it1->interior.min_latitude_edge.vertices[0] << " -> " << it1->interior.min_latitude_edge.vertices[1] << " | " << it1->interior.max_latitude_edge.vertices[0] << " -> " << it1->interior.max_latitude_edge.vertices[1] << std::endl;

                // Print out the immediate neighbors, the edges themselves, and the next one:
                std::cerr << "Immediate neighbors: " << std::endl;
                if (it1 != ranges.begin())
                {
                    auto prev = std::prev(it1);
                    std::cerr << "Prev range: " << prev->interior.min_latitude_edge.vertices[0] << " -> " << prev->interior.min_latitude_edge.vertices[1] << " | " << prev->interior.max_latitude_edge.vertices[0] << " -> " << prev->interior.max_latitude_edge.vertices[1] << std::endl;
                }
                std::cerr << "Range 1: " << it1->interior.min_latitude_edge.vertices[0] << " -> " << it1->interior.min_latitude_edge.vertices[1] << " | " << it1->interior.max_latitude_edge.vertices[0] << " -> " << it1->interior.max_latitude_edge.vertices[1] << std::endl;
                std::cerr << "Range 2: " << it2->interior.min_latitude_edge.vertices[0] << " -> " << it2->interior.min_latitude_edge.vertices[1] << " | " << it2->interior.max_latitude_edge.vertices[0] << " -> " << it2->interior.max_latitude_edge.vertices[1] << std::endl;
                if (std::next(it2) != ranges.end())
                {
                    auto next = std::next(it2);
                    std::cerr << "Next range: " << next->interior.min_latitude_edge.vertices[0] << " -> " << next->interior.min_latitude_edge.vertices[1] << " | " << next->interior.max_latitude_edge.vertices[0] << " -> " << next->interior.max_latitude_edge.vertices[1] << std::endl;
                }
                std::cerr << std::endl;

                if (it1 != ranges.begin())
                {
                    // Check for potential intersection with the previous range.
                    if (add_potential_edgecross(std::prev(it1)->interior, it1->interior))
                    {
                        std::cerr << "Added intersection event! 1" << std::endl;
                    }
                }
                if (std::next(it2) != ranges.end())
                {
                    auto next = std::next(it2)->interior;
                    assert(event_queue.top().longitude > event.longitude + DOUBLE_EPSILON);

                    // Check for potential intersection with the next range.
                    if (add_potential_edgecross(it2->interior, next))
                    {
                        std::cerr << "Added intersection event! 2" << std::endl;
                        assert(event_queue.top().longitude > event.longitude + DOUBLE_EPSILON);
                    }
                }

                // Check the order against the immediate neighbors:
                if (it1 != ranges.begin())
                {
                    assert(ranges.key_comp()(*std::prev(it1), *it1));
                }

                if (std::next(it2) != ranges.end())
                {
                    assert(ranges.key_comp()(*it2, *std::next(it2)));
                }

                // Set the current longitude halfway to the next event.
                if (!event_queue.empty())
                {
                    // TODO: Careful about angle wrapping nonsense.
                    double next_longitude = event_queue.top().longitude;

                    std::cerr << "Advancing longitude from " << event.longitude << " to " << (event.longitude + next_longitude) / 2.0 << std::endl;
                    assert(next_longitude >= event.longitude);

                    this->current_longitude = (event.longitude + next_longitude) / 2.0;
                }

                // Check the order between the swapped ranges:
                if (!ranges.key_comp()(*it1, *it2))
                {
                    assert(ranges.key_comp()(*it1, *it2));
                }


            }
            break;

        case 2: // EdgePairEnd
            {
                assert(std::get_if<EdgePairEnd>(&event.event) != nullptr);
                const LatitudeRangeBetweenEdges& range = std::get<EdgePairEnd>(event.event).range;

                // Find the range.
                auto it = ranges.find({range});

                // Sanity check: the range should be in the set.
                if (it != ranges.end()) {

                    // If before and after, add potential crossing.
                    if (it != ranges.begin() && std::next(it) != ranges.end())
                    {
                        add_potential_edgecross(std::prev(it)->interior, std::next(it)->interior);
                    }

                    // Remove the range from the set.
                    ranges.erase({range});

                    std::cerr << "Removed range: " << range.min_latitude_edge.vertices[0] << " -> " << range.min_latitude_edge.vertices[1] << " | " << range.max_latitude_edge.vertices[0] << " -> " << range.max_latitude_edge.vertices[1] << std::endl;
                } else
                {
                    std::cerr << "Range not found; likely because the corresponding insertion failed." << std::endl;
                }
            }
            break;

        default:
            throw std::runtime_error("Invalid event type");
        }

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
                    std::cerr << "Latitude at current longitude: " << ranges.key_comp().latitudeAtCurrentLongitude(prev->interior) << " vs "
                    << ranges.key_comp().latitudeAtCurrentLongitude(it->interior) << std::endl;
                }
                assert(ranges.key_comp()(*prev, *it));

                ++prev;
                ++it;
            }
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
