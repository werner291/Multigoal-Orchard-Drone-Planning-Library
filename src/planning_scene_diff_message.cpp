//
// Created by werner on 18-08-21.
//

#include <std_msgs/ColorRGBA.h>
#include "planning_scene_diff_message.h"

void createTrunkInPlanningSceneMessage(const std::vector<DetachedTreeNode> &tree_flattened,
                                       moveit_msgs::PlanningScene &planning_scene) {
    moveit_msgs::CollisionObject collision_object;

    collision_object.id = "trunk";
    collision_object.header.frame_id = "world";

    for (const auto &tree_node: tree_flattened) {
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

    planning_scene.world.collision_objects.push_back(collision_object);

    moveit_msgs::ObjectColor col;
    col.id = "trunk";
    col.color.a = 1.0;
    col.color.r = 0.5;
    col.color.g = 0.3;
    col.color.b = 0.1;
    planning_scene.object_colors.push_back(col);
}


void spawnLeavesInPlanningScene(const std::vector<Eigen::Vector3d> &leafVertices,
                                moveit_msgs::PlanningScene &planningScene) {
    moveit_msgs::CollisionObject leavesCollision;
    leavesCollision.id = "leaves";
    leavesCollision.header.frame_id = "world";

    shape_msgs::Mesh mesh;
    for (const auto &vtx: leafVertices) {
        geometry_msgs::Point pt;
        pt.x = vtx.x();
        pt.y = vtx.y();
        pt.z = vtx.z();
        mesh.vertices.push_back(pt);
    }

    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
        shape_msgs::MeshTriangle tri;
        tri.vertex_indices[0] = i;
        tri.vertex_indices[1] = i + 1;
        tri.vertex_indices[2] = i + 2;
        mesh.triangles.push_back(tri);
    }

    leavesCollision.meshes.push_back(mesh);
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    leavesCollision.mesh_poses.push_back(pose);

    // We allow collision with the leaves.
    planningScene.allowed_collision_matrix.default_entry_names.push_back("leaves");
    planningScene.allowed_collision_matrix.default_entry_values.push_back(false);

    planningScene.world.collision_objects.push_back(leavesCollision);

    moveit_msgs::ObjectColor leavesColor;
    leavesColor.id = "leaves";
    leavesColor.color.a = 1.0;
    leavesColor.color.r = 0.0;
    leavesColor.color.g = 0.8;
    leavesColor.color.b = 0.0;
    planningScene.object_colors.push_back(leavesColor);
}

void spawnApplesInPlanningScene(const double appleRadius, const std::vector<Apple> &apples,
                                moveit_msgs::PlanningScene &planning_scene_diff) {
    moveit_msgs::CollisionObject leavesCollision;
    leavesCollision.id = "apples";
    leavesCollision.header.frame_id = "world";

    for (const auto &apple: apples) {

        geometry_msgs::Pose pose;
        pose.position.x = apple.center.x();
        pose.position.y = apple.center.y();
        pose.position.z = apple.center.z();
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        leavesCollision.primitive_poses.push_back(pose);


        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = appleRadius;
        leavesCollision.primitives.push_back(primitive);
    }

    planning_scene_diff.world.collision_objects.push_back(leavesCollision);

    moveit_msgs::ObjectColor applesColor;
    applesColor.id = "apples";
    applesColor.color.a = 1.0;
    applesColor.color.r = 1.0;
    applesColor.color.g = 0.0;
    applesColor.color.b = 0.0;
    planning_scene_diff.object_colors.push_back(applesColor);
}

void spawnFloorInPlanningScene(moveit_msgs::PlanningScene &planning_scene_diff) {
    moveit_msgs::CollisionObject floorCollision;
    floorCollision.id = "floor";
    floorCollision.header.frame_id = "world";

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.5;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    floorCollision.primitive_poses.push_back(pose);

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 10.0;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 10.0;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
    floorCollision.primitives.push_back(primitive);

    planning_scene_diff.world.collision_objects.push_back(floorCollision);

    moveit_msgs::ObjectColor color;
    color.id = "floor";
    color.color.a = 1.0;
    color.color.r = 0.3;
    color.color.g = 0.2;
    color.color.b = 0.1;
    planning_scene_diff.object_colors.push_back(color);
}

moveit_msgs::PlanningScene
createPlanningSceneDiff(const std::vector<DetachedTreeNode> &treeFlattened,
                        const std::vector<Eigen::Vector3d> &leafVertices,
                        const double appleRadius, const std::vector<Apple> &apples) {
    moveit_msgs::PlanningScene planning_scene_diff;
    createTrunkInPlanningSceneMessage(treeFlattened, planning_scene_diff);
    spawnLeavesInPlanningScene(leafVertices, planning_scene_diff);
    spawnApplesInPlanningScene(appleRadius, apples, planning_scene_diff);
    spawnFloorInPlanningScene(planning_scene_diff);
    planning_scene_diff.is_diff = true;
    return planning_scene_diff;
}