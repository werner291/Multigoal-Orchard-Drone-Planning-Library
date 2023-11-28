//
// Created by werner on 28-11-23.
//

#include "latitude_sweep.h"

#include <queue>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

namespace mgodpl
{

    struct Range
    {
        double min, max;
    };

    /**
     * @brief   Compute the latitude of the given point, if projected onto a sphere centered at the given center.
     *
     * @param   point   The point to compute the latitude of.
     * @param   center  The center of the sphere.
     *
     * @return  The latitude, as a double in the range [-pi/2, pi/2].
     */
    double latitude(const math::Vec3d& point, const math::Vec3d& center)
    {
        math::Vec3d delta = point - center;

        double distance_xy = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

        return std::atan2(delta.z(), distance_xy);
    }

    enum VertexType
    {
        CLOSING, OPENING, INFLECTION
    };

    VertexType vertex_type(const Triangle& triangle, size_t vertex_index, const math::Vec3d& center)
    {

    }

    /**
     */
    void latitude_sweep(const std::vector<Triangle>& triangles, const math::Vec3d& center)
    {

        struct VertexEvent
        {
            double latitude;
            size_t triangle_index;
            size_t vertex_index;
        };

        std::vector<VertexEvent> vertex_events;

        // Create a vector of vertex events.
        for (size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index)
        {
            const Triangle& triangle = triangles[triangle_index];

            for (size_t vertex_index = 0; vertex_index < 3; ++vertex_index)
            {
               vertex_events.push_back(VertexEvent{
                   latitude(triangle.vertices[vertex_index], center),
                   triangle_index,
                   vertex_index
               });
            }
        }

        // Sort by latitude.
        std::sort(vertex_events.begin(), vertex_events.end(), [](const VertexEvent& a, const VertexEvent& b) {
            return a.latitude < b.latitude;
        });

        // Two possible cases: the first vertex is either a closing or an opening vertex.

        std::vector<Range> valid_ranges;

    }

    void longitude_sweep(const std::vector<Triangle>& triangles, const math::Vec3d& center)
    {

        struct VertexEvent
        {
            double latitude, longitude;
            size_t triangle_index, vertex_index;
        };

        std::vector<VertexEvent> vertex_events;

        for (size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index)
        {
            const Triangle& triangle = triangles[triangle_index];

            for (size_t vertex_index = 0; vertex_index < 3; ++vertex_index)
            {
                math::Vec3d delta = triangle.vertices[vertex_index] - center;

                vertex_events.push_back(VertexEvent{
                    .latitude=(std::atan2(delta.z(), std::sqrt(delta.x() * delta.x() + delta.y() * delta.y()))),
                    .longitude=(std::atan2(delta.y(), delta.x())),
                    .triangle_index=triangle_index,
                    .vertex_index=vertex_index
                });

            }
        }

        // sort by longitude
        std::sort(vertex_events.begin(), vertex_events.end(), [](const VertexEvent& a, const VertexEvent& b) {
            return a.longitude < b.longitude;
        });

        // Now, for longitude = 0, find all triangles intersected by the sweep-halfplane.

        std::vector<Range> valid_ranges;

        for (size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index)
        {
            // For every pair of vertices...
            for (int v_id : {0,1,2}) {
                math::Vec3d v1 = triangles[triangle_index].vertices[v_id] - center;
                math::Vec3d v2 = triangles[triangle_index].vertices[(v_id + 1) % 3] - center;

                // Find the intersection of the sweep-halfplane with the edge.
                // For longitude 0, the sweep-halfplane is the xz-plane, limited to the y >= 0 half-space.

                // First, make sure they've got opposing sign on the x-coordinate.
                if (v1.x() * v2.x() >= 0) {
                    continue;
                }




            }
        }

    }
}
