//
// Created by werner on 17-11-23.
//

#include <vtkProperty.h>
#include <random_numbers/random_numbers.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <vtkRenderer.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/detail/traversal/collision_node-inl.h>

#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/quick_markers.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/fcl_utils.h"
#include "../planning/RobotModel.h"

using namespace mgodpl;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

struct RobotState
{
    math::Transformd base_tf;
    std::vector<double> joint_values;
};

struct RobotVizualization
{
    std::vector<vtkSmartPointer<vtkActor>> actors;
};

bool check_link_collision(const mgodpl::robot_model::RobotModel::Link& link,
                          const fcl::CollisionObjectd& tree_trunk_object,
                          const std::vector<math::Transformd>::value_type& link_tf)
{
    bool collision = false;

    for (const auto& collision_geometry : link.collision_geometry)
    {
        if (const auto& box = std::get_if<Box>(&collision_geometry.shape))
        {
            math::Transformd total_tf = link_tf.then(collision_geometry.transform);

            fcl::Transform3d fcl_tf;
            fcl_tf.setIdentity();
            fcl_tf.translation() = fcl::Vector3d(total_tf.translation.x(), total_tf.translation.y(),
                                                 total_tf.translation.z());
            fcl_tf.rotate(fcl::Quaterniond(total_tf.orientation.w, total_tf.orientation.x,
                                           total_tf.orientation.y, total_tf.orientation.z));

            fcl::CollisionObjectd box_object(
                std::make_shared<fcl::Boxd>(box->size.x(), box->size.y(), box->size.z()),
                fcl_tf);

            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;
            fcl::collide(
                &tree_trunk_object,
                &box_object,
                request,
                result
            );

            if (result.isCollision())
            {
                collision = true;
                break;
            }
        }
        else
        {
            throw std::runtime_error("Only boxes are implemented for collision geometry.");
        }
    }

    return collision;
}

struct Array2d
{
    std::vector<double> data;
    size_t width;
    size_t height;

    std::vector<double>::reference operator()(size_t x, size_t y)
    {
        return data[y * width + x];
    }
};

//
// /**
//  * Generate a spherical "heighmap" around a given point.
//  *
//  * @param   center      The center of the heightmap.
//  * @param   max_radius  The maximum radius of the heightmap.
//  * @param   mesh        The mesh to wrap.
//  * @param   n_segments  The number of segments to use for the heightmap.
//  */
// Array2d generateHeightmap(const math::Vec3d& center, double max_radius, const Mesh& mesh, size_t n_segments)
// {
//     Array2d result;
//
//     result.width = n_segments;
//     result.height = n_segments;
//
//     result.data.resize(n_segments * n_segments, 0.0);
//
//     // For every vertex in the mesh (TODO: handle triangles properly), raise the heightmap to the height of the vertex.
//
//     for (const auto& vertex : mesh.vertices)
//     {
//         math::Vec3d delta = vertex - center;
//
//         // Convert to latitude and longitude.
//         double longitude = std::atan2(delta.y(), delta.x());
//         double latitude = std::atan2(delta.z(), std::sqrt(delta.x() * delta.x() + delta.y() * delta.y()));
//
//         // Remap to grid coordinates.
//         size_t x = std::clamp(static_cast<size_t>((longitude / (2.0 * M_PI) + 0.5) * (double) n_segments), size_t(0), n_segments - 1);
//         size_t y = std::clamp(static_cast<size_t>((latitude / M_PI + 0.5) * (double) n_segments), size_t(0), n_segments - 1);
//
//
//     }
//
// }

RobotState genUprightState(random_numbers::RandomNumberGenerator rng)
{
    RobotState state = {
        .base_tf = math::Transformd{
            .translation = math::Vec3d(0.0, 0.0, 0.0),
            .orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), rng.uniformReal(-M_PI, M_PI))
        },
        .joint_values = {rng.uniformReal(-M_PI / 2.0, M_PI / 2.0)}
    };

    return state;
}

RobotState genGoalState(
    random_numbers::RandomNumberGenerator rng,
    const math::Vec3d& target,
    const robot_model::RobotModel& robot,
    const robot_model::RobotModel::LinkId& flying_base,
    const robot_model::RobotModel::LinkId& end_effector)
{
    RobotState state = genUprightState(rng);

    const auto& fk = robot_model::forwardKinematics(
        robot,
        state.joint_values,
        flying_base,
        state.base_tf
    );

    math::Vec3d target_delta = target - fk.forLink(end_effector).translation;

    state.base_tf.translation = state.base_tf.translation + target_delta;

    return state;
}

void gen_color_coded_states(const mgodpl::robot_model::RobotModel& robot,
                            robot_model::RobotModel::LinkId flying_base,
                            robot_model::RobotModel::LinkId end_effector,
                            fcl::CollisionObjectd tree_trunk_object,
                            random_numbers::RandomNumberGenerator rng,
                            mgodpl::SimpleVtkViewer& viewer,
                            const std::vector<math::Vec3d>& targets,
                            size_t hardest_target)
{
    const size_t N_SAMPLES = 50;
    for (size_t i = 0; i < N_SAMPLES; ++i)
    {
        RobotState state = genUprightState(rng);

        const auto& fk = robot_model::forwardKinematics(
            robot,
            state.joint_values,
            flying_base,
            state.base_tf
        );

        math::Vec3d target_delta = targets[hardest_target] - fk.forLink(end_effector).translation;

        state.base_tf.translation = state.base_tf.translation + target_delta;

        const auto& fk_at_target = robot_model::forwardKinematics(
            robot,
            state.joint_values,
            flying_base,
            state.base_tf
        );

        for (size_t i = 0; i < fk_at_target.link_transforms.size(); ++i)
        {
            auto& link_tf = fk_at_target.link_transforms[i];

            bool collision = check_link_collision(robot.getLinks()[i], tree_trunk_object, link_tf);

            for (const auto& geometry : robot.getLinks()[i].visual_geometry.empty()
                                            ? robot.getLinks()[i].collision_geometry
                                            : robot.getLinks()[i].visual_geometry)
            {
                math::Vec3d color = collision ? math::Vec3d(1.0, 0.0, 0.0) : math::Vec3d(0.0, 1.0, 0.0);

                PositionedShape positioned_shape{
                    .shape = geometry.shape,
                    .transform = link_tf.then(geometry.transform)
                };

                viewer.addPositionedShape(positioned_shape, color);
            }
        }
    }
}

struct Range
{
    double min, max;
};

struct SortedIntervals
{

};

struct Triangle
{
    math::Vec3d a, b, c;
};



/**
 * @brief   Assuming a completely fixed stick, compute the ranges of the base Z-rotation that would not cause a collision.
 *
 * Note: this is just a 1D problem!
 */
std::vector<Range> fixed_stick_ranges(const mgodpl::robot_model::RobotModel& robot, const math::Vec3d& target, const Mesh& tree_mesh)
{



}



std::vector<double> compute_collision_probabilities(const mgodpl::robot_model::RobotModel& robot,
                                                    robot_model::RobotModel::LinkId flying_base,
                                                    robot_model::RobotModel::LinkId end_effector,
                                                    fcl::CollisionObjectd tree_trunk_object,
                                                    random_numbers::RandomNumberGenerator rng,
                                                    const std::vector<math::Vec3d>& targets)
{
    std::vector<double> collision_probabilities;

    for (const auto& target : targets)
    {
        assert(robot.count_joint_variables() == 1);

        const size_t N_SAMPLES = 1000;

        int n_collisions = 0;

        for (size_t i = 0; i < N_SAMPLES; ++i)
        {
            RobotState state = genUprightState(rng);

            const auto& fk = robot_model::forwardKinematics(
                robot,
                state.joint_values,
                flying_base,
                state.base_tf
            );

            math::Vec3d target_delta = target - fk.forLink(end_effector).translation;

            state.base_tf.translation = state.base_tf.translation + target_delta;

            const auto& fk_at_target = robot_model::forwardKinematics(
                robot,
                state.joint_values,
                flying_base,
                state.base_tf
            );

            for (size_t i = 0; i < fk_at_target.link_transforms.size(); ++i)
            {
                auto& link_tf = fk_at_target.link_transforms[i];

                bool collision = check_link_collision(robot.getLinks()[i], tree_trunk_object, link_tf);

                if (collision)
                {
                    n_collisions++;
                    break;
                }
            }
        }

        double collision_probability = (double)n_collisions / (double)N_SAMPLES;

        collision_probabilities.push_back(collision_probability);
    }
    return collision_probabilities;
}

int main()
{
    const auto& robot = mgodpl::experiments::createProceduralRobotModel();

    robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
    robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

    const auto& tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

    // Allocate a BVH mesh for the tree trunk.
    const auto& tree_trunk_bvh = mgodpl::fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
    fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

    random_numbers::RandomNumberGenerator rng(42);

    mgodpl::SimpleVtkViewer viewer;
    viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
    viewer.addMesh(createGroundPlane(5.0, 5.0), FLOOR_COLOR);

    const auto& targets = computeFruitPositions(tree_model);

    std::vector<double> collision_probabilities =
        compute_collision_probabilities(
            robot,
            flying_base,
            end_effector,
            tree_trunk_object,
            rng,
            targets);

    for (size_t i = 0; i < targets.size(); ++i)
    {
        math::Vec3d color =
            math::Vec3d(1.0, 0.0, 0.0) * collision_probabilities[i] +
            math::Vec3d(0.0, 1.0, 0.0) * (1.0 - collision_probabilities[i]);

        visualization::mkPointMarkerSphere(viewer, targets[i], color);
    }

    // Pick the hardest non-impossible target; that is, the target with the highest collision probability that is not 1.0.
    size_t hardest_target = 0;
    for (size_t i = 0; i < collision_probabilities.size(); ++i)
    {
        if (collision_probabilities[i] > collision_probabilities[hardest_target] && collision_probabilities[i] < 1.0)
        {
            hardest_target = i;
        }
    }

    std::cout
        << "Hardest target is " << hardest_target
		<< " at coords " << targets[hardest_target]
		<< " with collision probability " << collision_probabilities[hardest_target] << std::endl;

    {
        gen_color_coded_states(robot, flying_base, end_effector, tree_trunk_object, rng, viewer, targets,
                               hardest_target);

        viewer.addTimerCallback([&]()
        {
        });

        viewer.start();

        std::cout << "Done." << std::endl;
    }
}
