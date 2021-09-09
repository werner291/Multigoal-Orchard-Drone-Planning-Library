//
// Created by werner on 18-08-21.
//

#include <std_msgs/ColorRGBA.h>
#include "build_planning_scene.h"

PlanningScene establishPlanningScene(int branchingDepth, int numberOfApples) {

    std::vector<DetachedTreeNode> tree_flattened;
    make_tree_branches(Eigen::Isometry3d::Identity(), branchingDepth, 0.5, tree_flattened);

    PlanningScene planning_scene{};

    {
        moveit_msgs::CollisionObject collision_object;

        collision_object.id = "trunk";
        collision_object.header.frame_id = "world";

        for (const auto &tree_node : tree_flattened) {
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = tree_node.radius;
            primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = tree_node.length;

            collision_object.primitives.push_back(primitive);

            geometry_msgs::Pose pose;

            Eigen::Vector3d position = tree_node.root_at_absolute.translation();
            Eigen::Quaterniond quat(tree_node.root_at_absolute.rotation());

            position += quat * Eigen::Vector3d(0, 0, tree_node.length / 2.0);

            pose.position.x = position.x();
            pose.position.y = position.y();
            pose.position.z = position.z();

            pose.orientation.x = quat.x();
            pose.orientation.y = quat.y();
            pose.orientation.z = quat.z();
            pose.orientation.w = quat.w();

            collision_object.primitive_poses.push_back(pose);
        }

        planning_scene.moveit_diff.world.collision_objects.push_back(collision_object);

        moveit_msgs::ObjectColor col;
        col.id = "trunk";
        col.color.a = 1.0;
        col.color.r = 0.5;
        col.color.g = 0.3;
        col.color.b = 0.1;
        planning_scene.moveit_diff.object_colors.push_back(col);
    }

    {

        std::vector<Eigen::Vector3d> leaf_vertices = generateLeafVertices(tree_flattened);


        moveit_msgs::CollisionObject leaves_collision;
        leaves_collision.id = "leaves";
        leaves_collision.header.frame_id = "world";

        shape_msgs::Mesh mesh;
        for (const auto &vtx : leaf_vertices) {
            geometry_msgs::Point pt;
            pt.x = vtx.x();
            pt.y = vtx.y();
            pt.z = vtx.z();
            mesh.vertices.push_back(pt);
        }

        for (size_t i = 0; i < mesh.vertices.size();i+=3) {
            shape_msgs::MeshTriangle tri;
            tri.vertex_indices[0] = i;
            tri.vertex_indices[1] = i + 1;
            tri.vertex_indices[2] = i + 2;
            mesh.triangles.push_back(tri);
        }

        leaves_collision.meshes.push_back(mesh);
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        leaves_collision.mesh_poses.push_back(pose);

        // We allow collision with the leaves.
        planning_scene.moveit_diff.allowed_collision_matrix.default_entry_names.push_back("leaves");
        planning_scene.moveit_diff.allowed_collision_matrix.default_entry_values.push_back(false);

//        planning_scene.moveit_diff.world.collision_objects.push_back(leaves_collision);

        moveit_msgs::ObjectColor leaves_color;
        leaves_color.id = "leaves";
        leaves_color.color.a = 1.0;
        leaves_color.color.r = 0.0;
        leaves_color.color.g = 0.8;
        leaves_color.color.b = 0.0;
//        planning_scene.moveit_diff.object_colors.push_back(leaves_color);

        planning_scene.leaf_vertices = std::move(leaf_vertices);
    }

    {
        const double APPLE_RADIUS = 0.05;

        std::vector<Apple> apples = spawn_apples(tree_flattened, numberOfApples, APPLE_RADIUS);

        moveit_msgs::CollisionObject leaves_collision;
        leaves_collision.id = "apples";
        leaves_collision.header.frame_id = "world";

        for (const auto &apple : apples) {

            geometry_msgs::Pose pose;
            pose.position.x = apple.center.x();
            pose.position.y = apple.center.y();
            pose.position.z = apple.center.z();
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;
            leaves_collision.primitive_poses.push_back(pose);


            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = APPLE_RADIUS;
            leaves_collision.primitives.push_back(primitive);
        }

        planning_scene.moveit_diff.world.collision_objects.push_back(leaves_collision);

        moveit_msgs::ObjectColor apples_color;
        apples_color.id = "apples";
        apples_color.color.a = 1.0;
        apples_color.color.r = 1.0;
        apples_color.color.g = 0.0;
        apples_color.color.b = 0.0;
        planning_scene.moveit_diff.object_colors.push_back(apples_color);

        planning_scene.apples = apples;
    }

    {

        moveit_msgs::CollisionObject floor_collision;
        floor_collision.id = "floor";
        floor_collision.header.frame_id = "world";

        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = -0.5;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        floor_collision.primitive_poses.push_back(pose);


        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 10.0;
        primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 10.0;
        primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
        floor_collision.primitives.push_back(primitive);

        planning_scene.moveit_diff.world.collision_objects.push_back(floor_collision);

        moveit_msgs::ObjectColor color;
        color.id = "floor";
        color.color.a = 1.0;
        color.color.r = 0.3;
        color.color.g = 0.2;
        color.color.b = 0.1;
        planning_scene.moveit_diff.object_colors.push_back(color);
    }

    planning_scene.moveit_diff.is_diff = true;

    return planning_scene;
}