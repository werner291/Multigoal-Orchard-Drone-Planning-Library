//
// Created by werner on 28-11-23.
//

#include "latitude_sweep.h"

#include <queue>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/filter.hpp>
#include <iostream>
#include <optional>

namespace mgodpl
{
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

    PolarCoordinates polar_coordinates(const math::Vec3d& point, const math::Vec3d& center)
    {
        return PolarCoordinates{latitude(point, center), longitude(point, center), (point - center).norm()};
    }

    std::array<size_t, 3> triangle_vertices_by_longitude(const Triangle& triangle, const math::Vec3d& center)
    {
        // Create an array to store the original vertex indices: {0, 1, 2}.
        std::array<size_t, 3> vertex_indices = {0, 1, 2};

        // Sort the vertex indices based on their longitude values.
        std::sort(vertex_indices.begin(), vertex_indices.end(), [&](size_t a, size_t b)
        {
            // Calculate the longitude of each vertex relative to the specified center.
            double l1 = longitude(triangle.vertices[a], center);
            double l2 = longitude(triangle.vertices[b], center);

            // Use the sign og the signed longitude difference to determine the order.
            return signed_longitude_difference(l1, l2) < 0;
        });

        // Return the reordered vertex indices based on longitude.
        return vertex_indices;
    }

	double longitude_ahead_angle(const double starting_longitude, const double longitude);

	std::vector<VertexEvent> longitude_sweep_events(const std::vector<Triangle> &triangles,
													const math::Vec3d &center,
													const double starting_longitude)
    {

		// Create a vector to store the events.
        std::vector<VertexEvent> vertex_events;
		vertex_events.reserve(triangles.size() * 3);

        // For each triangle...
        for (size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index)
        {
            const Triangle& triangle = triangles[triangle_index];

            // Sort the indices of the triangle vertices by longitude.
            std::array<size_t, 3> vertex_indices = triangle_vertices_by_longitude(triangle, center);

            // Then, create three events: one for each vertex, in order of longitude.
            vertex_events.push_back(VertexEvent{
                .type = TRIANGLE_INTERSECTION_START,
                .triangle_index = triangle_index,
                .vertex_index = vertex_indices[0],
                .longitude = longitude(triangle.vertices[vertex_indices[0]], center),
				.relative_longitude = longitude_ahead_angle(starting_longitude, longitude(triangle.vertices[vertex_indices[0]], center))
            });

            vertex_events.push_back(VertexEvent{
                .type = INTERNAL_TRIANGLE_VERTEX,
                .triangle_index = triangle_index,
                .vertex_index = vertex_indices[1],
                .longitude = longitude(triangle.vertices[vertex_indices[1]], center),
				.relative_longitude = longitude_ahead_angle(starting_longitude, longitude(triangle.vertices[vertex_indices[1]], center))
            });

            vertex_events.push_back(VertexEvent{
                .type = TRIANGLE_INTERSECTION_END,
                .triangle_index = triangle_index,
                .vertex_index = vertex_indices[2],
                .longitude = longitude(triangle.vertices[vertex_indices[2]], center),
				.relative_longitude = longitude_ahead_angle(starting_longitude, longitude(triangle.vertices[vertex_indices[2]], center))
            });
        }

        // sort by longitude; be careful about wrapping.
        std::sort(vertex_events.begin(), vertex_events.end(), [&](const VertexEvent& a, const VertexEvent& b)
        {
			return a.relative_longitude < b.relative_longitude;
        });

        return vertex_events;
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

    double segment_intersection_latitude(const math::Vec3d& segment_start, const math::Vec3d& segment_end,
                                         double sweep_longitude, const math::Vec3d& center)
    {
        // First, compute the lat/lon of the segment start and end projected onto the sphere.
        const double start_longitude = longitude(segment_start, center);
        const double start_latitude = latitude(segment_start, center);
        const double end_longitude = longitude(segment_end, center);
        const double end_latitude = latitude(segment_end, center);

        // Check if the segment is in the right order.
        assert(signed_longitude_difference(end_longitude, start_longitude) > 0);

        // Check that the intersection exists.
        assert(signed_longitude_difference(sweep_longitude, start_longitude) >= 0);
        assert(signed_longitude_difference(end_longitude, sweep_longitude) >= 0);

        // Compute the interpolation factor. (TODO: check if this is correct; should there be some idea of linearization?)
        const double interpolation_factor = signed_longitude_difference(sweep_longitude, start_longitude) /
            signed_longitude_difference(end_longitude, start_longitude);

        // Interpolate the latitude (this doesn't wrap since latitudes are in the range [-pi/2, pi/2]).
        return start_latitude + interpolation_factor * (end_latitude - start_latitude);
    }

	PolarCoordinates segment_intersection(
			const math::Vec3d& segment_start,
			const math::Vec3d& segment_end,
			double sweep_longitude,
			const math::Vec3d& center) {

		// First, compute the lat/lon of the segment start and end projected onto the sphere.
		const double start_longitude = longitude(segment_start, center);
		const double start_latitude = latitude(segment_start, center);
		const double end_longitude = longitude(segment_end, center);
		const double end_latitude = latitude(segment_end, center);

		// Check if the segment is in the right order.
		assert(signed_longitude_difference(end_longitude, start_longitude) > 0);

		// Check that the intersection exists.
		assert(signed_longitude_difference(sweep_longitude, start_longitude) >= 0);
		assert(signed_longitude_difference(end_longitude, sweep_longitude) >= 0);

		// Compute the interpolation factor. (TODO: check if this is correct; should there be some idea of linearization?)
		const double interpolation_factor = signed_longitude_difference(sweep_longitude, start_longitude) /
											signed_longitude_difference(end_longitude, start_longitude);

		double start_radius = (segment_start - center).norm();
		double end_radius = (segment_end - center).norm();



		return {
				start_latitude + interpolation_factor * (end_latitude - start_latitude),
				sweep_longitude,
				start_radius + interpolation_factor * (end_radius - start_radius)
		};


	}

    std::array<ArcSegmentIntersection, 2> current_segment_intersections(const TriangleIntersection& intersection,
                                                                        double longitude)
    {
        // Precondition check: the longitude should be in the range of the intersection.
        assert(signed_longitude_difference(longitude, intersection.opening_longitude) >= 0);

        // Check whether we're before or after the inflection point.
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
        // Precondition check: the longitude should be in the range of the intersection.
        assert(signed_longitude_difference(longitude, intersection.opening_longitude) >= 0);
        assert(signed_longitude_difference(longitude, intersection.closing_longitude) <= 0);

        // First, compute the current segment intersections.
        const auto& [i1,i2] = current_segment_intersections(intersection, longitude);

        // Then, compute the latitudes of the intersections.
        double latitude1 = segment_intersection_latitude(
            triangles[i1.triangle_index].vertices[i1.edge_start_vertex_index],
            triangles[i1.triangle_index].vertices[i1.edge_end_vertex_index],
            longitude,
            center
        );

        double latitude2 = segment_intersection_latitude(
            triangles[i2.triangle_index].vertices[i2.edge_start_vertex_index],
            triangles[i2.triangle_index].vertices[i2.edge_end_vertex_index],
            longitude,
            center
        );

        return {latitude1, latitude2};
    }


	std::array<PolarCoordinates, 2> intersection_arc(const TriangleIntersection &intersection,
													 double longitude,
													 const math::Vec3d &center,
													 const std::vector<Triangle> &triangles) {

		// Precondition check: the longitude should be in the range of the intersection.
		assert(signed_longitude_difference(longitude, intersection.opening_longitude) >= 0);
		assert(signed_longitude_difference(longitude, intersection.closing_longitude) <= 0);

		// First, compute the current segment intersections.
		const auto& [i1,i2] = current_segment_intersections(intersection, longitude);

		// Then, compute the latitudes of the intersections.
		const auto& int1 = segment_intersection(
				triangles[i1.triangle_index].vertices[i1.edge_start_vertex_index],
				triangles[i1.triangle_index].vertices[i1.edge_end_vertex_index],
				longitude,
				center
		);

		const auto& int2 = segment_intersection(
				triangles[i2.triangle_index].vertices[i2.edge_start_vertex_index],
				triangles[i2.triangle_index].vertices[i2.edge_end_vertex_index],
				longitude,
				center
		);

		if (int1.latitude > int2.latitude) {
			return {
					int2,
					int1
			};
		} else {
			return {
					int1,
					int2
			};
		}

	}

	std::array<double, 2> vertical_padded_latitude_range(
			const TriangleIntersection& intersection,
			double longitude,
			double vertical_padding,
			const math::Vec3d& center,
			const std::vector<Triangle>& triangles
	) {

		auto [a,b] = intersection_arc(intersection, longitude, center, triangles);

		double padding_a = std::atan(vertical_padding / a.radius);
		double padding_b = std::atan(vertical_padding / b.radius);

		return {
				a.latitude - padding_a,
				b.latitude + padding_b
		};

	}

    OngoingIntersections triangle_intersections(const std::vector<Triangle>& triangles, const math::Vec3d& center,
                                                const double sweep_longitude)
    {
		OngoingIntersections intersections;
		intersections.intersections.resize(triangles.size(), std::nullopt);

        for (size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index)
        {
            const Triangle& triangle = triangles[triangle_index];

            // Sort by longitude.
            std::array<size_t, 3> vertex_indices = triangle_vertices_by_longitude(triangle, center);

            const double opening_longitude = longitude(triangle.vertices[vertex_indices[0]], center);
            const double closing_longitude = longitude(triangle.vertices[vertex_indices[2]], center);

            // Sanity check: make sure closing longitude is greater than opening longitude.
            assert(signed_longitude_difference(closing_longitude, opening_longitude) > 0);

            // If the longitude is not in the range, skip this triangle.
            if (signed_longitude_difference(sweep_longitude, opening_longitude) >= 0 &&
                signed_longitude_difference(sweep_longitude, closing_longitude) <= 0)
            {
                // Create an intersection.
				intersections.intersections[triangle_index]= TriangleIntersection {
                    .triangle_index = triangle_index,
                    .opening_vertex_index = vertex_indices[0],
                    .closing_vertex_index = vertex_indices[2],
                    .inflection_vertex_index = vertex_indices[1],
                    .opening_longitude = opening_longitude,
                    .closing_longitude = closing_longitude,
                    .inflection_longitude = longitude(triangle.vertices[vertex_indices[1]], center)
                };
            }
        }

        return intersections;
    }

    void update_intersections(OngoingIntersections& intersections, const VertexEvent& event,
                              const std::vector<Triangle>& triangles, const math::Vec3d& center)
    {
        switch (event.type)
        {
        case TRIANGLE_INTERSECTION_START:
            {
                // Sort the triangle vertices (TODO: duplicating this a bit; can we cache this?)
                std::array<size_t, 3> vertex_indices = triangle_vertices_by_longitude(triangles[event.triangle_index], center);

                // A new triangle intersection starts; add it to the list.
                intersections.intersections[event.triangle_index] = {
                    .triangle_index = event.triangle_index,
                    .opening_vertex_index = vertex_indices[0],
                    .closing_vertex_index = vertex_indices[2],
                    .inflection_vertex_index = vertex_indices[1],
                    .opening_longitude = longitude(triangles[event.triangle_index].vertices[vertex_indices[0]], center),
                    .closing_longitude = longitude(triangles[event.triangle_index].vertices[vertex_indices[2]], center),
                    .inflection_longitude = longitude(triangles[event.triangle_index].vertices[vertex_indices[1]],
                                                      center)
                };
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
                intersections.intersections[event.triangle_index] = std::nullopt;
            }
            break;
        }
    }

    std::vector<std::array<double, 2>> free_latitude_ranges(const OngoingIntersections &intersections,
															const math::Vec3d &center,
															double sweep_longitude,
															const std::vector<Triangle> &triangles,
															double vertical_padding)
    {


		// Step 1: compute the occupied list of latitude ranges.
        std::vector<std::array<double, 2>> occupied_latitude_ranges = intersections.intersections |
				// Skip the tombstones.
				ranges::views::filter([](const auto& intersection) { return intersection.has_value(); }) |
				// Unwrap the optionals.
				ranges::views::transform([](const auto& intersection) { return intersection.value(); }) |
            	ranges::views::transform(
                [&](const TriangleIntersection& intersection)
                {
                    auto range = vertical_padded_latitude_range(intersection, sweep_longitude, vertical_padding, center, triangles);
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

    OrderedPolarTriangle polar_triangle(const Triangle& triangle, const math::Vec3d& center)
    {
        std::array<PolarCoordinates, 3> vertices{
            polar_coordinates(triangle.vertices[0], center),
            polar_coordinates(triangle.vertices[1], center),
            polar_coordinates(triangle.vertices[2], center)
        };

        // Sort by longitude; compare by signed longitude difference.
        if (signed_longitude_difference(vertices[0].longitude, vertices[1].longitude) > 0)
        {
            std::swap(vertices[0], vertices[1]);
        }

        return {
            .vertices = vertices
        };
    }

	double longitude_ahead_angle(const double starting_longitude, const double longitude) {
		double a_longitude = signed_longitude_difference(longitude, starting_longitude);
		if (a_longitude < 0) a_longitude += 2 * M_PI;
		return a_longitude;
	}

	LongitudeSweep::LongitudeSweep(const std::vector<Triangle> &triangles,
								   const math::Vec3d &target,
								   double starting_longitude) :
			triangles(triangles),
			intersections(triangle_intersections(triangles, target, starting_longitude)),
			events(longitude_sweep_events(triangles, target, starting_longitude)),
			target(target),
			starting_longitude(starting_longitude),
			events_passed(0) {}

	std::array<double, 2> LongitudeSweep::current_longitude_range() {
		if (events_passed == 0) {
			return {starting_longitude, events[0].longitude};
		} else if (events_passed == events.size()) {
			return {events.back().longitude, starting_longitude + 2 * M_PI};
		} else {
			return {events[events_passed - 1].longitude, events[events_passed].longitude};
		}
	}

	const OngoingIntersections &LongitudeSweep::current_intersections() const {
		return intersections;
	}

	size_t LongitudeSweep::number_of_ranges() const {
		return events.size() + 1;
	}

	size_t LongitudeSweep::ranges_passed() const {
		return events_passed;
	}

	void LongitudeSweep::advance() {
		assert(has_more_ranges());
		do {
			update_intersections(intersections, events[events_passed++], triangles, target);
		} while (events_passed < events.size() && events[events_passed].relative_longitude == events[events_passed - 1].relative_longitude);
	}

	bool LongitudeSweep::has_more_ranges() const {
		return ranges_passed() < number_of_ranges();
	}
}
