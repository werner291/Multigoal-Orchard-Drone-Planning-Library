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

            return true;
        }
        else
        {
            return false;
        }
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

            if (on_first_edge)
            {
                // Check to make sure we're on the longitude range:
                assert(in_longitude_range(edges.short_1, current_longitude));
                assert(in_longitude_range(edges.long_edge, current_longitude));
                ranges.insert(rg1);
            }

            if (on_second_edge)
            {
                // Check to make sure we're on the longitude range:
                assert(in_longitude_range(edges.short_2, current_longitude));
                assert(in_longitude_range(edges.long_edge, current_longitude));
                ranges.insert(rg2);
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

    void LongitudeSweep::process_next_event()
    {
        // Grab the next event:
        const SweepEvent event = event_queue.top();
        event_queue.pop();

        std::cout << "Rel lon: " << event.relative_longitude << " with " << event_queue.size() << " events remaining." << std::endl;

        switch (event.event.index())
        {
        case 0: // EdgePairStart
            {
                assert(std::get_if<EdgePairStart>(&event.event) != nullptr);
                const LatitudeRangeBetweenEdges& range = std::get<EdgePairStart>(event.event).range;

                this->current_longitude = event.longitude + DOUBLE_EPSILON;

                auto it_already_in = ranges.find(range);

                if (it_already_in != ranges.end())
                {
                    std::cout << "Already in!" << std::endl;

                    std::cout << "New range: " << range.min_latitude_edge.vertices[0] << " -> " << range.min_latitude_edge.vertices[1] << " | " << range.max_latitude_edge.vertices[0] << " -> " << range.max_latitude_edge.vertices[1] << std::endl;
                    std::cout << "Old range: " << it_already_in->min_latitude_edge.vertices[0] << " -> " << it_already_in->min_latitude_edge.vertices[1] << " | " << it_already_in->max_latitude_edge.vertices[0] << " -> " << it_already_in->max_latitude_edge.vertices[1] << std::endl;
                }

                // Insert the range into the set.
                const auto& [it, inserted] = ranges.insert(range);

                assert(inserted);

                if (it != ranges.begin())
                {
                    // Check for potential intersection with the previous range.
                    const auto& prev_range = *std::prev(it);

                    add_potential_edgecross(prev_range, range);
                }

                if (std::next(it) != ranges.end())
                {
                    // Check for potential intersection with the next range.
                    const auto& next_range = *std::next(it);

                    add_potential_edgecross(range, next_range);
                }

                // TODO: add/remove intersection events.
            }
            break;
        case 1: // EdgePairSwap
            {
                assert(std::get_if<EdgePairSwap>(&event.event) != nullptr);
                const LatitudeRangeBetweenEdges& range1 = std::get<EdgePairSwap>(event.event).range1;
                const LatitudeRangeBetweenEdges& range2 = std::get<EdgePairSwap>(event.event).range2;

                // Wow this is ugly... TODO Does that const value work properly? Do we need to do a dynamic epsilon?
                this->current_longitude = event.longitude - DOUBLE_EPSILON;

                double l1pre = this->comparator.latitudeAtCurrentLongitude(range1);
                double l2pre = this->comparator.latitudeAtCurrentLongitude(range2);

                if (!this->comparator(range2, range1))
                {
                    std::cout << "Ranges out of order: " << std::endl;
                    std::cout << "Range 1: " << range1.min_latitude_edge.vertices[0] << " -> " << range1.min_latitude_edge.vertices[1] << " | " << range1.max_latitude_edge.vertices[0] << " -> " << range1.max_latitude_edge.vertices[1] << std::endl;
                    std::cout << "Range 2: " << range2.min_latitude_edge.vertices[0] << " -> " << range2.min_latitude_edge.vertices[1] << " | " << range2.max_latitude_edge.vertices[0] << " -> " << range2.max_latitude_edge.vertices[1] << std::endl;
                }

                assert(this->comparator(range2, range1));

                {
                    auto it1 = ranges.find(range1);
                    auto it2 = ranges.find(range2);

                    // Ensure that it2 is just after it1.
                    if(std::next(it1) != it2)
                    {
                        // Event was stale.
                        break;
                    }
                }

                // Delete them from the set.
                ranges.erase(range1);
                ranges.erase(range2);

                this->current_longitude = event.longitude + DOUBLE_EPSILON;

                double l1 = this->comparator.latitudeAtCurrentLongitude(range1);
                double l2 = this->comparator.latitudeAtCurrentLongitude(range2);
                if (!this->comparator(range2, range1))
                {

                    std::cout << "L1: " << this->comparator.latitudeAtCurrentLongitude(range1) << std::endl;
                    std::cout << "L2: " << this->comparator.latitudeAtCurrentLongitude(range2) << std::endl;
                }

                if (!this->comparator(range2, range1))
                {
                    std::cout << "Ranges out of order: " << std::endl;
                    std::cout << "Range 1: " << range1.min_latitude_edge.vertices[0] << " -> " << range1.min_latitude_edge.vertices[1] << " | " << range1.max_latitude_edge.vertices[0] << " -> " << range1.max_latitude_edge.vertices[1] << std::endl;
std::cout << "Range 2: " << range2.min_latitude_edge.vertices[0] << " -> " << range2.min_latitude_edge.vertices[1] << " | " << range2.max_latitude_edge.vertices[0] << " -> " << range2.max_latitude_edge.vertices[1] << std::endl;
                }

                // Check to make sure the comparator puts range2 after range1:
                assert(this->comparator(range2, range1));

                {
                    // Note: do not use these iterators because the second insert invalidates the first one.
                    const auto& [_it1, rg1_inserted] = ranges.insert(range1);
                    const auto& [_it2, rg2_inserted] = ranges.insert(range2);

                    assert(rg1_inserted);
                    assert(rg2_inserted);
                }

                auto it1 = ranges.find(range1);
                auto it2 = ranges.find(range2);

                assert(ranges.key_comp().sweep->current_longitude == this->current_longitude);

                // Make sure that the order is now flipped:
                assert(std::next(it2) == it1);

                // FIXME: only add swap events after currnet longitude; also, things might be swapped below this line...

                this->current_longitude = event.longitude;


                // TODO: The edges now have new neighbors. Check for intersections with those.

                if (it2 != ranges.begin())
                {
                    // Check for potential intersection with the previous range.
                    const auto& prev_range = *std::prev(it2);
                    if (add_potential_edgecross(prev_range, range2))
                    {
                        std::cout << "Added intersection event!" << std::endl;
                    }
                }

                if (std::next(it1) != ranges.end())
                {
                    // Check for potential intersection with the next range.
                    const auto& next_range = *std::next(it1);
                    if (add_potential_edgecross(range1, next_range))
                    {
                        std::cout << "Added intersection event!" << std::endl;
                    }
                }
            }
            break;

        case 2: // EdgePairEnd
            {
                assert(std::get_if<EdgePairEnd>(&event.event) != nullptr);
                const LatitudeRangeBetweenEdges& range = std::get<EdgePairEnd>(event.event).range;

                this->current_longitude = event.longitude - DOUBLE_EPSILON;

                // Remove the range from the set.
                ranges.erase(range);

                // TODO remove intersection events (if applicable? Those should have already triggered by now.) (Or add ones for neighbors that now get close)
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
