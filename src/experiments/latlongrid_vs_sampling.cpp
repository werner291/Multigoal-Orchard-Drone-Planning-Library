/*
 * The purpose of this experiment is to get hard numbers on the performance of the lat/lon grid vs. sampling naively.
 */

#include <random_numbers/random_numbers.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/detail/traversal/collision_node-inl.h>
#include <chrono>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/fcl_utils.h"
#include "../planning/LatitudeLongitudeGrid.h"
#include "../planning/collision_detection.h"
#include "../planning/RelativeLatLonGrid.h"
#include "../planning/goal_sampling.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../visualization/VtkTriangleSetVisualization.h"

using namespace mgodpl;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};


RobotState from_arm_angles_and_target(const math::Vec3d& vec, const math::Vec3d& target, const robot_model::RobotModel& robot)
{

    // Generate a state.
    std::vector arm_angles{-spherical_geometry::latitude(vec)};

    math::Transformd flying_base_tf{
        .translation = math::Vec3d(0.0, 0.0, 0.0),
        .orientation = math::Quaterniond::fromAxisAngle(
            math::Vec3d::UnitZ(), spherical_geometry::longitude(vec) + M_PI / 2.0)
    };

    const auto& fk = mgodpl::robot_model::forwardKinematics(robot, arm_angles, 0, flying_base_tf);

    const math::Vec3d end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;

    flying_base_tf.translation = flying_base_tf.translation + target - end_effector_position;

    RobotState st{
        .base_tf = flying_base_tf,
        .joint_values = arm_angles
    };

    return st;

}

std::vector<math::Vec3d> gen_free_arm_vectors(const RelativeLatLonGrid& grid, random_numbers::RandomNumberGenerator rng)
{
    std::vector<math::Vec3d> free_arm_vectors;

    for (size_t lat_i = 0; lat_i < grid.grid.latitude_cells; ++lat_i)
    {
        for (size_t lon_i = 0; lon_i < grid.grid.longitude_cells; ++lon_i)
        {
            auto cell = grid.grid.cells[grid.grid.cell_index({lat_i, lon_i})];

            if (cell.triangles.empty())
            {
                // Generate a Lat/lon inside the cell.
                auto lats = grid.grid.latitude_range_of_cell(lat_i);
                auto lons = grid.grid.longitude_range_of_cell(lon_i);

                double lat = lats.interpolate(rng.uniform01());
                double lon = lons.interpolate(rng.uniform01()).longitude;

                // Convert to an arm vec.
                mgodpl::math::Vec3d vec = grid.to_global.orientation.rotate(
                    spherical_geometry::RelativeVertex{lon, lat}.to_cartesian());

                free_arm_vectors.push_back(vec);
            }
        }
    }

    return free_arm_vectors;
}

std::optional<RobotState> findGoalStateThroughLatLonGrid(
    const math::Vec3d& target,
    const math::Vec3d& canopy_middle,
    const robot_model::RobotModel& robot,
    const robot_model::RobotModel::LinkId& flying_base,
    const robot_model::RobotModel::LinkId& stick,
    const robot_model::RobotModel::LinkId& end_effector,
    const fcl::CollisionObjectd& tree_trunk_object,
    random_numbers::RandomNumberGenerator rng,
    const std::vector<Triangle>& triangles
)
{

    RelativeLatLonGrid relative_grid = from_center_and_ideal_vector(canopy_middle, target - canopy_middle);

    for (const auto& triangle : triangles)
    {
        // If all three vertices are behind the ideal vector, skip this triangle.
        if ((triangle.vertices[0] - target).dot(target - canopy_middle) < 0.0 &&
            (triangle.vertices[1] - target).dot(target - canopy_middle) < 0.0 &&
            (triangle.vertices[2] - target).dot(target - canopy_middle) < 0.0)
        {
            continue;
        }

        // Drop triangles that are further than STICK_LENGTH from the target.
        if ((triangle.vertices[0] - target).norm() > experiments::STICK_LENGTH &&
            (triangle.vertices[1] - target).norm() > experiments::STICK_LENGTH &&
            (triangle.vertices[2] - target).norm() > experiments::STICK_LENGTH)
        {
            continue;
        }

        insert_triangle(relative_grid, triangle);
    }

    for (const auto& vec : gen_free_arm_vectors(relative_grid, rng))
    {
        // Generate a state.
        std::vector<double> arm_angles{-spherical_geometry::latitude(vec)};

        math::Transformd flying_base_tf{
            .translation = math::Vec3d(0.0, 0.0, 0.0),
            .orientation = math::Quaterniond::fromAxisAngle(
                math::Vec3d::UnitZ(), spherical_geometry::longitude(vec) + M_PI / 2.0)
        };

        const auto& fk = mgodpl::robot_model::forwardKinematics(
            robot, arm_angles, flying_base, flying_base_tf);

        math::Vec3d end_effector_position = fk.forLink(end_effector).translation;

        flying_base_tf.translation = flying_base_tf.translation + target - end_effector_position;

        RobotState st{
            .base_tf = flying_base_tf,
            .joint_values = arm_angles
        };

        bool collision = check_robot_collision(robot, tree_trunk_object, st);

        if (!collision)
        {
            return st;
        }
    }

    return std::nullopt;
}

std::array<math::Vec3d, 4> compute_cell_grid_corners(const RelativeLatLonGrid& grid, size_t lat_i, size_t lon_i, double radius = 0.1)
{
    auto lats = grid.grid.latitude_range_of_cell(lat_i);
    auto lons = grid.grid.longitude_range_of_cell(lon_i);

    // Get the four corners of the cell.
    math::Vec3d v1 = grid.to_global.apply(
        spherical_geometry::RelativeVertex{lons.start, lats.min}.to_cartesian() * radius);
    math::Vec3d v2 = grid.to_global.apply(
        spherical_geometry::RelativeVertex{lons.end, lats.min}.to_cartesian() * radius);
    math::Vec3d v3 = grid.to_global.apply(
        spherical_geometry::RelativeVertex{lons.end, lats.max}.to_cartesian() * radius);
    math::Vec3d v4 = grid.to_global.apply(
        spherical_geometry::RelativeVertex{lons.start, lats.max}.to_cartesian() * radius);

    return {v1, v2, v3, v4};
}

void vizualize_grid_occupancy(SimpleVtkViewer& viewer, const RelativeLatLonGrid& grid)
{
    std::vector<std::array<math::Vec3d, 3>> grid_triangles_fully_occupied;
    std::vector<std::array<math::Vec3d, 3>> grid_triangles_occupied;
    std::vector<std::array<math::Vec3d, 3>> grid_triangles_empty;

    for (size_t lat_i = 0; lat_i < grid.grid.latitude_cells; ++lat_i)
    {
        for (size_t lon_i = 0; lon_i < grid.grid.longitude_cells; ++lon_i)
        {
            // Get the four corners of the cell.
            const auto& [v1, v2, v3, v4] = compute_cell_grid_corners(grid, lat_i, lon_i);

            // Make two triangles:
            std::array<math::Vec3d, 3> t1{v1, v2, v3};
            std::array<math::Vec3d, 3> t2{v1, v3, v4};

            // if the cell is empty, put into one; else, put into the other.
            if (grid.grid.cells[grid.grid.cell_index({lat_i, lon_i})].triangles.empty())
            {
                grid_triangles_empty.push_back(t1);
                grid_triangles_empty.push_back(t2);
            }
            else
            {
                grid_triangles_occupied.push_back(t1);
                grid_triangles_occupied.push_back(t2);
            }
        }
    }

    VtkTriangleSetVisualization fully_occupied_visualization(1.0, 0.0, 0.0, 0.8);
    VtkTriangleSetVisualization occupied_visualization(1.0, 0.0, 0.5, 0.8);
    VtkTriangleSetVisualization empty_visualization(0.0, 1.0, 0.5, 0.8);

    fully_occupied_visualization.updateTriangles(grid_triangles_fully_occupied);
    occupied_visualization.updateTriangles(grid_triangles_occupied);
    empty_visualization.updateTriangles(grid_triangles_empty);

    // viewer.addActor(fully_occupied_visualization.getActor());
    // viewer.addActor(occupied_visualization.getActor());
    viewer.addActor(empty_visualization.getActor());
}

void vizualize_grid_triangle_connections(SimpleVtkViewer &viewer, const RelativeLatLonGrid &grid)
{
    std::vector<std::pair<math::Vec3d, math::Vec3d>> cell_to_triangles;
    for (size_t lat_i = 0; lat_i < grid.grid.latitude_cells; ++lat_i)
    {
        for (size_t lon_i = 0; lon_i < grid.grid.longitude_cells; ++lon_i)
        {
            const auto& [v1, v2, v3, v4] = compute_cell_grid_corners(grid, lat_i, lon_i);
            math::Vec3d cell_center = (v1 + v2 + v3 + v4) / 4.0;

            for (const auto& triangle : grid.grid.cells[grid.grid.cell_index({lat_i, lon_i})].triangles)
            {
                // Bring the triangle back to global transform:
                math::Vec3d triangle_center = (triangle.vertices[0] + triangle.vertices[1] + triangle.vertices[2]) /
                    3.0;

                triangle_center = grid.to_global.apply(triangle_center);

                cell_to_triangles.emplace_back(cell_center, triangle_center);
            }
        }
    }

    VtkLineSegmentsVisualization cell_to_triangle_visualization(0.0, 0.0, 1.0);
    cell_to_triangle_visualization.updateLine(cell_to_triangles);
    viewer.addActor(cell_to_triangle_visualization.getActor());
}

void visualize_apples(const std::vector<math::Vec3d>& targets, SimpleVtkViewer& viewer)
{
    for (const auto& target : targets)
    {
        viewer.addSphere(0.05, target, {1.0, 0.0, 0.0}, 1.0); // Red color for apples
    }
}

std::vector<mgodpl::Triangle> trunk_to_triangles(const tree_meshes::TreeMeshes& tree_model)
{

    std::vector<mgodpl::Triangle> triangles;

    for (const auto& triangle : tree_model.trunk_mesh.triangles)
    {
        triangles.push_back({
            .vertices = {
                math::Vec3d{
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].x,
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].y,
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].z
                },
                math::Vec3d{
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].x,
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].y,
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].z
                },
                math::Vec3d{
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].x,
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].y,
                    tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].z
                }
            }
        });
    }

    return triangles;
}

void vizualize_arm_positions(const mgodpl::robot_model::RobotModel& robot,
    robot_model::RobotModel::LinkId flying_base,
    robot_model::RobotModel::LinkId stick,
    fcl::CollisionObjectd& tree_trunk_object,
    random_numbers::RandomNumberGenerator& rng,
    SimpleVtkViewer &viewer,
    const math::Vec3<double>& target,
    RelativeLatLonGrid grid)
{
    const auto& arm_vectors = gen_free_arm_vectors(grid, rng);

    // Now, sample some robot states and visualize them:
    for (const auto& vec: arm_vectors)
    {
        RobotState st = from_arm_angles_and_target(vec, target, robot);

        const auto& fk2 = mgodpl::robot_model::forwardKinematics(
            robot, st.joint_values, flying_base, st.base_tf);

        PositionedShape positioned_shape{
            .shape = robot.getLinks()[stick].collision_geometry[0].shape,
            .transform = fk2.forLink(stick).then(robot.getLinks()[stick].collision_geometry[0].transform)
        };

        // Check single link collision.
        bool collision = check_link_collision(robot.getLinks()[stick], tree_trunk_object, st.base_tf);

        if (!collision)
        {
            viewer.addPositionedShape(positioned_shape, {0.0, 0.5, 1.0}, 1.0);
        }
        else
        {
            viewer.addPositionedShape(positioned_shape, {1.0, 0.0, 0.0}, 1.0);
        }

    }
}

// Define interaction style.
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

    std::optional<std::function<void(const std::string& key)>> key_callback;

    virtual void OnKeyPress() override
    {
        // Get the keypress.
        vtkRenderWindowInteractor* rwi = this->Interactor;
        std::string key = rwi->GetKeySym();

        if (key_callback.has_value())
        {
            (*key_callback)(key);
        }

        // Forward events.
        vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
};
vtkStandardNewMacro(KeyPressInteractorStyle);

int main()
{
    const auto& robot = mgodpl::experiments::createProceduralRobotModel();

    robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
    robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");
    robot_model::RobotModel::LinkId stick = robot.findLinkByName("stick");

    const auto& tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

    std::vector<Triangle> triangles = trunk_to_triangles(tree_model);

    // Allocate a BVH mesh for the tree trunk.
    const auto& tree_trunk_bvh = mgodpl::fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
    fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

    random_numbers::RandomNumberGenerator rng(42);

    const auto& targets = computeFruitPositions(tree_model);

    math::Vec3d canopy_middle = mesh_aabb(tree_model.leaves_mesh).center();

    size_t uniform_successes = 0;
    size_t grid_successes = 0;

    // Tally the total durations:
    long uniform_duration = 0;
    long grid_duration = 0;

    std::vector<bool> uniform_found;
    std::vector<bool> grid_found;

    // Let's examine, for every target, how hard sampling is through either method.
    for (size_t i = 0; i < targets.size(); ++i)
    {
        std::cout << "Target " << i << " is at " << targets[i] << std::endl;

        math::Vec3d target = targets[i];

        auto start = std::chrono::high_resolution_clock::now();

        bool found_uniform = findGoalStateByUniformSampling(
            target,
            robot,
            flying_base,
            end_effector,
            tree_trunk_object,
            rng,
            1000
        ).has_value();

        auto end = std::chrono::high_resolution_clock::now();
        uniform_duration += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        auto grid_start = std::chrono::high_resolution_clock::now();
        // And now through the grid.
        bool found_grid = findGoalStateThroughLatLonGrid(
            target,
            canopy_middle,
            robot,
            flying_base,
            stick,
            end_effector,
            tree_trunk_object,
            rng,
            triangles
        ).has_value();
        auto grid_end = std::chrono::high_resolution_clock::now();

        grid_duration += std::chrono::duration_cast<std::chrono::milliseconds>(grid_end - grid_start).count();

        std::cout << "Uniform sampling: " << (found_uniform ? "found" : "not found") << std::endl;
        std::cout << "Grid sampling: " << (found_grid ? "found" : "not found") << std::endl;

        if (found_uniform)
        {
            ++uniform_successes;
        }

        if (found_grid)
        {
            ++grid_successes;
        }

        uniform_found.push_back(found_uniform);
        grid_found.push_back(found_grid);

        // Stats so far:
        std::cout << "Uniform sampling successes: " << uniform_successes << " with t = " << uniform_duration << " ms" <<
            std::endl;
        std::cout << "Grid sampling successes: " << grid_successes << " with t = " << grid_duration << " ms" <<
            std::endl;
    }

    std::cout << "Uniform sampling successes: " << uniform_successes << "/" << targets.size() << std::endl;
    std::cout << "Grid sampling successes: " << grid_successes << "/" << targets.size() << std::endl;

    SimpleVtkViewer viewer;

    viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

    math::Vec3d canopy_center = mesh_aabb(tree_model.leaves_mesh).center();

    for (size_t i = 0; i < targets.size(); ++i)
    {
        double r = uniform_found[i] ? 1.0 : 0.0;
        double b = grid_found[i] ? 1.0 : 0.0;

        viewer.addSphere(0.05, targets[i], {r, 0.0, b}, 0.5);
    }

    for (const auto& target : targets)
    {
        // Now, let's try to visualize the lat/lon grid.
        math::Vec3d ideal_vector = target - canopy_center;

        // Create a grid:
        RelativeLatLonGrid grid = from_center_and_ideal_vector(target, ideal_vector);

        // Check that the target is (0,0,0) in the local coordinate system:
        assert((grid.transform.apply(target) - math::Vec3d(0.0, 0.0, 0.0)).norm() < 1e-6);

        // Check that a (0,0,0) in the local coordinate system is the target:
        assert((grid.to_global.apply(math::Vec3d(0.0, 0.0, 0.0)) - target).norm() < 1e-6);

        // Insert the triangles:
        for (const auto& triangle : triangles)
        {
            insert_triangle(grid, triangle);
        }

        vizualize_grid_occupancy(viewer, grid);
        // vizualize_grid_triangle_connections(viewer, grid);
        // vizualize_arm_positions(robot, flying_base, stick, tree_trunk_object, rng, viewer, target, grid);
    }

    size_t target_i = 0;
    double lat = 0.0;
    double lon = 0.0;

    // auto selected_apple = viewer.addSphere(0.06, targets[target_i], {1.0,1.0,0.0}, 0.5);
    //
    // vtkNew<KeyPressInteractorStyle> style;
    //
    // style->key_callback = [&](const std::string& key)
    // {
    //     if (key == "Up")
    //     {
    //         target_i = (target_i + 1) % targets.size();
    //         selected_apple->SetPosition(targets[target_i].x(), targets[target_i].y(), targets[target_i].z());
    //     }
    //
    //     if (key == "Down")
    //     {
    //         target_i = (target_i - 1) % targets.size();
    //         selected_apple->SetPosition(targets[target_i].x(), targets[target_i].y(), targets[target_i].z());
    //     }
    // };

    // viewer.renderWindowInteractor->SetInteractorStyle(style);

    viewer.addTimerCallback([&]()
    {

    });

    viewer.start();
}
