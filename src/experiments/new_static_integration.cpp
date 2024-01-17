// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>
#include <vtkActor.h>

#include "../math/Transform.h"
#include "../math/Vec3.h"
#include "../planning/RobotModel.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/fcl_utils.h"
#include "../planning/ConvexHullSpace.h"
#include "../planning/goal_sampling.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkTriangleSetVisualization.h"

using namespace mgodpl;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

struct RobotActors
{
    std::vector<vtkSmartPointer<vtkActor>> link_actors;

    RobotActors(const robot_model::RobotModel& robot_model, const robot_model::ForwardKinematicsResult& fk, SimpleVtkViewer& viewer)
    {
        for (size_t link_i = 0; link_i < fk.link_transforms.size(); ++link_i)
        {
            if (!robot_model.getLinks()[link_i].visual_geometry.empty())
            {
                PositionedShape shape = robot_model.getLinks()[link_i].visual_geometry[0];
                shape.transform = fk.link_transforms[link_i].then(shape.transform);
                link_actors.push_back(viewer.addPositionedShape(shape, {0.5,0.5,0.5}, 1));
            } else if (!robot_model.getLinks()[link_i].collision_geometry.empty()) {
                PositionedShape shape = robot_model.getLinks()[link_i].collision_geometry[0];
                shape.transform = fk.link_transforms[link_i].then(shape.transform);
                link_actors.push_back(viewer.addPositionedShape(shape, {0.5,0.5,0.5}, 1));
            }
        }
    }

    void apply_fk(const robot_model::RobotModel& robot_model, const robot_model::ForwardKinematicsResult& fk)
    {
        auto actor_it = link_actors.begin();
        for (size_t link_i = 0; link_i < fk.link_transforms.size(); ++link_i)
        {
            if (!robot_model.getLinks()[link_i].visual_geometry.empty())
            {
                auto [_shape, transform] = robot_model.getLinks()[link_i].visual_geometry[0];
                transform = fk.link_transforms[link_i].then(transform);
                SimpleVtkViewer::set_transform(transform, actor_it->Get());
            } else if (!robot_model.getLinks()[link_i].collision_geometry.empty()) {
                auto [_shape, transform] = robot_model.getLinks()[link_i].collision_geometry[0];
                transform = fk.link_transforms[link_i].then(transform);
                SimpleVtkViewer::set_transform(transform, actor_it->Get());
            }
            ++actor_it;
        }
    }
};

bool check_motion_collision(
    const robot_model::RobotModel& robot_model,
    const fcl::CollisionObjectd& tree_trunk_object,
    const RobotState& start,
    const RobotState& goal)
{

    const auto& fk_start = robot_model::forwardKinematics(
        robot_model,
        start.joint_values,
        robot_model.findLinkByName("flying_base"),
        start.base_tf
    );

    const auto& fk_goal = robot_model::forwardKinematics(
        robot_model,
        goal.joint_values,
        robot_model.findLinkByName("flying_base"),
        goal.base_tf
    );

    for (size_t link_id = 0; link_id < robot_model.getLinks().size(); ++link_id)
    {
        const auto& link = robot_model.getLinks()[link_id];

        if (link.collision_geometry.empty())
        {
            continue;
        }

        const auto& link_transform_start = fk_start.link_transforms[link_id];
        const auto& link_transform_goal = fk_goal.link_transforms[link_id];

        for (const auto& [shape, transform]: link.collision_geometry)
        {
            math::Transformd total_transform_start = link_transform_start.then(transform);
            math::Transformd total_transform_goal = link_transform_goal.then(transform);

        }
    }

}

void visualize_chull(const ConvexHullShellSpace& shell, SimpleVtkViewer& viewer)
{
    const auto& mesh = shell.get_mesh();

    VtkTriangleSetVisualization chull(1.0,1.0,1.0,0.5);

    std::vector<std::array<math::Vec3d, 3>> triangles;

    for (const auto& face: mesh.faces())
    {
        auto vertex = vertices_around_face(mesh.halfedge(face), mesh).begin();
        auto pt1 = mesh.point(*(vertex++));
        auto pt2 = mesh.point(*(vertex++));
        auto pt3 = mesh.point(*(vertex));

        triangles.push_back({
            math::Vec3d {pt1.x(), pt1.y(), pt1.z()},
            math::Vec3d {pt2.x(), pt2.y(), pt2.z()},
            math::Vec3d {pt3.x(), pt3.y(), pt3.z()}
        });
    }

    chull.updateTriangles(triangles);

    viewer.addActor(chull.getActor());
}

int main()
{

    const auto& robot = experiments::createProceduralRobotModel();

    robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
    robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");
    robot_model::RobotModel::LinkId stick = robot.findLinkByName("stick");

    const auto& tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Allocate a BVH mesh for the tree trunk.
    fcl::CollisionObjectd tree_trunk_object(fcl_utils::meshToFclBVH(tree_model.trunk_mesh));

    const auto& targets = computeFruitPositions(tree_model);

    // Build a chull shell.
    ConvexHullShellSpace shell(
        tree_model.leaves_mesh.vertices | ranges::views::transform([](const auto& v) {
            return math::Vec3d{v.x, v.y, v.z};
        }) | ranges::to<std::vector>()
    );




    random_numbers::RandomNumberGenerator rng;

    std::vector<std::optional<std::array<RobotState, 2>>> solutions;

    // For all targets...
    for (const auto& target: targets)
    {
        const auto& target_projection = shell.closestPoint(target);

        auto goal_state = findGoalStateByUniformSampling(
            target,
            robot,
            flying_base,
            end_effector,
            tree_trunk_object,
            rng,
            1000);

        if (!goal_state.has_value())
        {
            solutions.push_back(std::nullopt);
            continue;
        }

        const auto &fk = robot_model::forwardKinematics(
            robot,
            goal_state->joint_values,
            robot.findLinkByName("flying_base"),
            goal_state->base_tf
        );

        auto arm_vec = fk.link_transforms[robot.findLinkByName("stick")].orientation.rotate(math::Vec3d(0,-1,0));

        RobotState outside_tree = *goal_state;
        outside_tree.base_tf.translation = outside_tree.base_tf.translation + arm_vec * 0.5;

        solutions.emplace_back(std::array<RobotState, 2>{
            *goal_state,
            outside_tree
        });
    }

    SimpleVtkViewer viewer;

    viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
    for (size_t target_i = 0; target_i < targets.size(); ++target_i)
    {
        viewer.addSphere(0.05, targets[target_i], {1, 0, 0}, 1.0);

        if (solutions[target_i].has_value())
        {
            RobotActors actors1(robot, forwardKinematics(
                                    robot,
                                    solutions[target_i]->at(0).joint_values,
                                    flying_base,
                                    solutions[target_i]->at(0).base_tf
                                ), viewer);

            RobotActors actors2(robot, forwardKinematics(
                                    robot,
                                    solutions[target_i]->at(1).joint_values,
                                    flying_base,
                                    solutions[target_i]->at(1).base_tf
                                ), viewer);
        }
        break;
    }

    visualize_chull(shell, viewer);

    viewer.start();

}
