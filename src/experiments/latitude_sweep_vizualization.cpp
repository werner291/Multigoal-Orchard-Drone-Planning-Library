//
// Created by werner on 29-11-23.
//

#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

#include "../experiment_utils/TreeMeshes.h"
#include "../visualization/quick_markers.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkPolyLineVisualization.h"

#include "../planning/latitude_sweep.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../visualization/VtkTriangleSetVisualization.h"

using namespace mgodpl;

int main(int argc, char** argv)
{

    const math::Vec3d WOOD_COLOR = {0.5, 0.35, 0.05};

    // Get a tree model.
    const auto& model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

    const math::Vec3d target = {0.2, 0.5, 3.0};

    SimpleVtkViewer viewer;
    viewer.lockCameraUp();
    viewer.setCameraTransform(target + math::Vec3d{1.0, 0.0, 0.0}, target);

    visualization::mkPointMarkerSphere(viewer, target, {1.0, 0.0, 0.0});

    viewer.addMesh(
        model.trunk_mesh,
        WOOD_COLOR
    );

    VtkPolyLineVisualization line_visualization(1.0,0.0,1.0);
    viewer.addActor(line_visualization.getActor());

    VtkTriangleSetVisualization intersections_visualization(1.0, 1.0, 0.0);
    viewer.addActor(intersections_visualization.getActor());

    const std::vector<Triangle> triangles = model.trunk_mesh.triangles | ranges::views::transform(
        [&](const auto& triangle)
        {
            return Triangle{
                {
                    math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[0]].x,
                                model.trunk_mesh.vertices[triangle.vertex_indices[0]].y,
                                model.trunk_mesh.vertices[triangle.vertex_indices[0]].z),
                    math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[1]].x,
                                model.trunk_mesh.vertices[triangle.vertex_indices[1]].y,
                                model.trunk_mesh.vertices[triangle.vertex_indices[1]].z),
                    math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[2]].x,
                                model.trunk_mesh.vertices[triangle.vertex_indices[2]].y,
                                model.trunk_mesh.vertices[triangle.vertex_indices[2]].z)
                }
            };
        }) | ranges::to<std::vector>();

    double longitude = M_PI;

    viewer.addTimerCallback([&]()
    {

        longitude += 0.0005;

        std::vector<math::Vec3d> points;
        for (int lat_i = 0; lat_i <= 32; ++lat_i)
        {
            // Latitude from [-pi/2, pi/2]
            double latitude = -M_PI / 2.0 + lat_i * M_PI / 32.0;

            math::Vec3d ray(
                cos(latitude) * cos(longitude),
                cos(latitude) * sin(longitude),
                sin(latitude)
            );

            points.push_back(target + ray * 1.0);
        }
        line_visualization.updateLine(points);

        const auto& intersections = triangle_intersections(
            triangles,
            target,
            longitude
        );

        std::vector<std::array<math::Vec3d, 3>> intersection_points;
        for (const auto& latitudes : free_latitude_ranges(intersections, target, longitude, triangles))
        {
            double length = latitudes[1] - latitudes[0];

            size_t n_points = std::max(1, (int) (length * 16.0));

            for (int lat_i = 0; lat_i < n_points; ++lat_i)
            {
                double latitude1 = latitudes[0] + lat_i * (latitudes[1] - latitudes[0]) / (double) n_points;
                double latitude2 = latitudes[0] + (lat_i + 1) * (latitudes[1] - latitudes[0]) / (double) n_points;

                math::Vec3d ray1(
                    cos(latitude1) * cos(longitude),
                    cos(latitude1) * sin(longitude),
                    sin(latitude1)
                );

                math::Vec3d ray2(
                    cos(latitude2) * cos(longitude),
                    cos(latitude2) * sin(longitude),
                    sin(latitude2)
                );

                std::array<math::Vec3d, 3> triangle_points{
                    target + ray1 * 1.0,
                    target + ray2 * 1.0,
                    target
                };

                intersection_points.push_back(triangle_points);
            }
        }

        intersections_visualization.updateTriangles(intersection_points);

    });

    viewer.start();


}