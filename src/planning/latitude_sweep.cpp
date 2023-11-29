//
// Created by werner on 28-11-23.
//

#include "latitude_sweep.h"

#include <queue>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

namespace mgodpl
{
    double latitude(const math::Vec3d& point, const math::Vec3d& center)
    {
        math::Vec3d delta = point - center;

        double distance_xy = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

        return std::atan2(delta.z(), distance_xy);
    }

    double longitude(const math::Vec3d& point, const math::Vec3d& center)
    {
        math::Vec3d delta = point - center;

        return std::atan2(delta.y(), delta.x());
    }

    std::array<size_t, 3> triangle_vertex_types(const Triangle& triangle, const math::Vec3d& center)
    {
        // Sort by longitude.
        std::array<size_t, 3> vertex_indices = {0, 1, 2};
        std::sort(vertex_indices.begin(), vertex_indices.end(), [&](size_t a, size_t b)
        {
            double l1 = longitude(triangle.vertices[a], center);
            double l2 = longitude(triangle.vertices[b], center);

            return signed_longitude_difference(l1, l2) < 0;
        });
        return vertex_indices;
    }

    std::vector<VertexEvent> longitude_sweep_events(const std::vector<Triangle>& triangles, const math::Vec3d& center)
    {
        std::vector<VertexEvent> vertex_events;

        for (size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index)
        {
            const Triangle& triangle = triangles[triangle_index];

            // Sort by longitude.
            std::array<size_t, 3> vertex_indices = triangle_vertex_types(triangle, center);

            // Then, create three events: one for each vertex, in order of longitude.
            vertex_events.push_back(VertexEvent{
                .type = TRIANGLE_INTERSECTION_START,
                .triangle_index = triangle_index,
                .vertex_index = vertex_indices[0],
                .longitude = longitude(triangle.vertices[vertex_indices[0]], center)
            });

            vertex_events.push_back(VertexEvent{
                .type = INTERNAL_TRIANGLE_VERTEX,
                .triangle_index = triangle_index,
                .vertex_index = vertex_indices[1],
                .longitude = longitude(triangle.vertices[vertex_indices[1]], center)
            });

            vertex_events.push_back(VertexEvent{
                .type = TRIANGLE_INTERSECTION_END,
                .triangle_index = triangle_index,
                .vertex_index = vertex_indices[2],
                .longitude = longitude(triangle.vertices[vertex_indices[2]], center)
            });
        }

        // sort by longitude
        std::sort(vertex_events.begin(), vertex_events.end(), [](const VertexEvent& a, const VertexEvent& b)
        {
            return a.longitude < b.longitude;
        });

        return vertex_events;
    }

    double signed_longitude_difference(double first, double second)
    {
        // Compute the difference:
        double difference = first - second;

        // Put it into the range [-pi, pi] (TODO: check if this is correct; the wrapping is confusing me)
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

    double segment_intersection_latitude(const math::Vec3d& segment_start, const math::Vec3d& segment_end,
                                         double sweep_longitude, const math::Vec3d& center)
    {
        // First, compute the lat/lon of the segment start and end projected onto the sphere.
        double start_longitude = longitude(segment_start, center);
        double start_latitude = latitude(segment_start, center);
        double end_longitude = longitude(segment_end, center);
        double end_latitude = latitude(segment_end, center);

        // Check if the segment is in the right order.
        assert(signed_longitude_difference(end_longitude, start_longitude) > 0);

        // Check that the intersection exists.
        assert(signed_longitude_difference(sweep_longitude, start_longitude) >= 0);
        assert(signed_longitude_difference(end_longitude, sweep_longitude) >= 0);

        // Compute the interpolation factor. (TODO: check if this is correct; should there be some idea of linearization?)
        double interpolation_factor = signed_longitude_difference(sweep_longitude, start_longitude) /
            signed_longitude_difference(end_longitude, start_longitude);

        // Interpolate the latitude (this doesn't wrap since latitudes are in the range [-pi/2, pi/2]).
        return start_latitude + interpolation_factor * (end_latitude - start_latitude);
    }

    std::array<ArcSegmentIntersection, 2> current_segment_intersections(const TriangleIntersection& intersection,
                                                                        double longitude)
    {
        assert(longitude >= intersection.opening_longitude);

        if (signed_longitude_difference(longitude, intersection.inflection_longitude) <= 0)
        {
            // The intersections are with the edges (opening, inflection), (opening, closing).
            return {
                ArcSegmentIntersection{
                    .triangle_index = intersection.triangle_index,
                    .edge_start_vertex_index = intersection.opening_vertex_index,
                    .edge_end_vertex_index = intersection.inflection_vertex_index
                },
                ArcSegmentIntersection{
                    .triangle_index = intersection.triangle_index,
                    .edge_start_vertex_index = intersection.opening_vertex_index,
                    .edge_end_vertex_index = intersection.closing_vertex_index
                }
            };
        }
        else
        {
            // The intersections are with the edges (opening, closing), (inflection, closing).
            return {
                ArcSegmentIntersection{
                    .triangle_index = intersection.triangle_index,
                    .edge_start_vertex_index = intersection.opening_vertex_index,
                    .edge_end_vertex_index = intersection.closing_vertex_index
                },
                ArcSegmentIntersection{
                    .triangle_index = intersection.triangle_index,
                    .edge_start_vertex_index = intersection.inflection_vertex_index,
                    .edge_end_vertex_index = intersection.closing_vertex_index
                }
            };
        }
    }

    std::array<double, 2> latitude_range(const TriangleIntersection& intersection, double longitude,
                                         const math::Vec3d& center, const std::vector<Triangle>& triangles)
    {
        assert(signed_longitude_difference(longitude, intersection.opening_longitude) >= 0);
        assert(signed_longitude_difference(longitude, intersection.closing_longitude) <= 0);

        // First, compute the current segment intersections.
        std::array<ArcSegmentIntersection, 2> intersections = current_segment_intersections(intersection, longitude);

        // Then, compute the latitudes of the intersections.
        double latitude1 = segment_intersection_latitude(
            triangles[intersections[0].triangle_index].vertices[intersections[0].edge_start_vertex_index],
            triangles[intersections[0].triangle_index].vertices[intersections[0].edge_end_vertex_index],
            longitude,
            center
        );

        double latitude2 = segment_intersection_latitude(
            triangles[intersections[1].triangle_index].vertices[intersections[1].edge_start_vertex_index],
            triangles[intersections[1].triangle_index].vertices[intersections[1].edge_end_vertex_index],
            longitude,
            center
        );

        return {latitude1, latitude2};
    }

    OngoingIntersections triangle_intersections(const std::vector<Triangle>& triangles, const math::Vec3d& center,
                                                const double sweep_longitude)
    {
        std::vector<TriangleIntersection> intersections;

        for (size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index)
        {
            const Triangle& triangle = triangles[triangle_index];

            // Sort by longitude.
            std::array<size_t, 3> vertex_indices = triangle_vertex_types(triangle, center);

            const double opening_longitude = longitude(triangle.vertices[vertex_indices[0]], center);
            const double closing_longitude = longitude(triangle.vertices[vertex_indices[2]], center);

            // Sanity check: make sure closing longitude is greater than opening longitude.
            assert(signed_longitude_difference(closing_longitude, opening_longitude) > 0);

            // If the longitude is not in the range, skip this triangle.
            if (signed_longitude_difference(sweep_longitude, opening_longitude) >= 0 &&
                signed_longitude_difference(sweep_longitude, closing_longitude) <= 0)
            {
                // Create an intersection.
                intersections.push_back(TriangleIntersection{
                    .triangle_index = triangle_index,
                    .opening_vertex_index = vertex_indices[0],
                    .closing_vertex_index = vertex_indices[2],
                    .inflection_vertex_index = vertex_indices[1],
                    .opening_longitude = opening_longitude,
                    .closing_longitude = closing_longitude,
                    .inflection_longitude = longitude(triangle.vertices[vertex_indices[1]], center)
                });
            }
        }

        return {
            .intersections = intersections
        };
    }

    void update_intersections(OngoingIntersections& intersections, const VertexEvent& event,
                              const std::vector<Triangle>& triangles, const math::Vec3d& center)
    {
        switch (event.type)
        {
        case TRIANGLE_INTERSECTION_START:
            {
                // Sort the triangle vertices (TODO: duplicating this a bit; can we cache this?)
                std::array<size_t, 3> vertex_indices = triangle_vertex_types(triangles[event.triangle_index], center);

                // A new triangle intersection starts; add it to the list.
                intersections.intersections.push_back({
                    .triangle_index = event.triangle_index,
                    .opening_vertex_index = vertex_indices[0],
                    .closing_vertex_index = vertex_indices[2],
                    .inflection_vertex_index = vertex_indices[1],
                    .opening_longitude = longitude(triangles[event.triangle_index].vertices[vertex_indices[0]], center),
                    .closing_longitude = longitude(triangles[event.triangle_index].vertices[vertex_indices[2]], center),
                    .inflection_longitude = longitude(triangles[event.triangle_index].vertices[vertex_indices[1]],
                                                      center)
                });
            }
            break;
        case INTERNAL_TRIANGLE_VERTEX:
            {
                // Do nothing; we only care about the start and end of the intersection.
                // Alternatively, we could keep a list of segment pair intersections, and update them as we go?
            }
            break;
        case TRIANGLE_INTERSECTION_END:
            {
                // Find the intesection for the given triangle index and delete it.
                // TODO: do better than linear search.
                for (size_t i = 0; i < intersections.intersections.size(); ++i)
                {
                    if (intersections.intersections[i].triangle_index == event.triangle_index)
                    {
                        intersections.intersections.erase(intersections.intersections.begin() + (long)i);
                        break;
                    }
                }
            }
            break;
        }
    }

    std::vector<std::array<double, 2>> free_latitude_ranges(
        const OngoingIntersections& intersections,
        const math::Vec3d& center,
        double sweep_longitude,
        const std::vector<Triangle>& triangles)
    {
        // Step 1: compute the occupied list of latitude ranges.
        std::vector<std::array<double, 2>> occupied_latitude_ranges = intersections.intersections | ranges::views::transform(
            [&](const TriangleIntersection& intersection)
            {
                auto range = latitude_range(intersection, sweep_longitude, center, triangles);
                if (range[0] > range[1])
                {
                    std::swap(range[0], range[1]);
                }
                return range;
            }) | ranges::to<std::vector>();

        // Step 2: sort by start latitude.
        std::sort(occupied_latitude_ranges.begin(), occupied_latitude_ranges.end(), [](const auto& a, const auto& b)
        {
            return a[0] < b[0];
        });

        // Step 3: merge overlapping ranges.
        std::vector<std::array<double, 2>> merged_latitude_ranges;
        for (const auto& range : occupied_latitude_ranges)
        {
            if (merged_latitude_ranges.empty())
            {
                merged_latitude_ranges.push_back(range);
            }
            else
            {
                if (range[0] <= merged_latitude_ranges.back()[1])
                {
                    // Overlapping ranges; merge.
                    merged_latitude_ranges.back()[1] = std::max(merged_latitude_ranges.back()[1], range[1]);
                }
                else
                {
                    // Non-overlapping ranges; add.
                    merged_latitude_ranges.push_back(range);
                }
            }
        }

        // Step 4: take the complement of the merged ranges.
        std::vector<std::array<double, 2>> free_latitude_ranges;
        if (merged_latitude_ranges.empty())
        {
            // If there are no ranges, the entire sphere is free.
            free_latitude_ranges.push_back({-M_PI / 2.0, M_PI / 2.0});
        }
        else
        {
            // Otherwise, the first range starts at the bottom of the sphere.
            if (merged_latitude_ranges.front()[0] > -M_PI / 2.0)
            {
                free_latitude_ranges.push_back({-M_PI / 2.0, merged_latitude_ranges.front()[0]});
            }

            // Then, add the ranges in between.
            for (size_t i = 0; i < merged_latitude_ranges.size() - 1; ++i)
            {
                free_latitude_ranges.push_back({merged_latitude_ranges[i][1], merged_latitude_ranges[i + 1][0]});
            }

            // Then, add the last range.
            if (merged_latitude_ranges.back()[1] < M_PI / 2.0)
            {
                free_latitude_ranges.push_back({merged_latitude_ranges.back()[1], M_PI / 2.0});
            }
        }

        return free_latitude_ranges;
    }
}
