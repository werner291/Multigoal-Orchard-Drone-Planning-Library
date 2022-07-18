#include <bullet/HACD/hacdHACD.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "../src/msgs_utilities.h"
#include "../src/experiment_utils.h"

#include "planning_scene_diff_message.h"
#include "general_utilities.h"
#include "Seb.h"

void createTrunkInPlanningSceneMessage(const std::vector<DetachedTreeNode> &tree_flattened,
                                       moveit_msgs::msg::PlanningScene &planning_scene) {
    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.id = "trunk";
    collision_object.header.frame_id = "world";

    for (const auto &tree_node: tree_flattened) {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = tree_node.radius;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = tree_node.length;

        collision_object.primitives.push_back(primitive);

        geometry_msgs::msg::Pose pose;

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

    moveit_msgs::msg::ObjectColor col;
    col.id = "trunk";
    col.color.a = 1.0;
    col.color.r = 0.5;
    col.color.g = 0.3;
    col.color.b = 0.1;
    planning_scene.object_colors.push_back(col);
}


void spawnLeavesInPlanningScene(const std::vector<Eigen::Vector3d> &leafVertices,
                                moveit_msgs::msg::PlanningScene &planningScene) {
    moveit_msgs::msg::CollisionObject leavesCollision;
    leavesCollision.id = "leaves";
    leavesCollision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    for (const auto &vtx: leafVertices) {
        geometry_msgs::msg::Point pt;
        pt.x = vtx.x();
        pt.y = vtx.y();
        pt.z = vtx.z();
        mesh.vertices.push_back(pt);
    }

    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
        shape_msgs::msg::MeshTriangle tri;
        tri.vertex_indices[0] = i;
        tri.vertex_indices[1] = i + 1;
        tri.vertex_indices[2] = i + 2;
        mesh.triangles.push_back(tri);
    }

    leavesCollision.meshes.push_back(mesh);
    geometry_msgs::msg::Pose pose;
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

    moveit_msgs::msg::ObjectColor leavesColor;
    leavesColor.id = "leaves";
    leavesColor.color.a = 1.0;
    leavesColor.color.r = 0.0;
    leavesColor.color.g = 0.8;
    leavesColor.color.b = 0.0;
    planningScene.object_colors.push_back(leavesColor);
}

void spawnApplesInPlanningScene(const double appleRadius, const std::vector<Apple> &apples,
                                moveit_msgs::msg::PlanningScene &planning_scene_diff) {
    moveit_msgs::msg::CollisionObject leavesCollision;
    leavesCollision.id = "apples";
    leavesCollision.header.frame_id = "world";

    for (const auto &apple: apples) {

        geometry_msgs::msg::Pose pose;
        pose.position.x = apple.center.x();
        pose.position.y = apple.center.y();
        pose.position.z = apple.center.z();
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        leavesCollision.primitive_poses.push_back(pose);


        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = appleRadius;
        leavesCollision.primitives.push_back(primitive);
    }

    planning_scene_diff.world.collision_objects.push_back(leavesCollision);

    moveit_msgs::msg::ObjectColor applesColor;
    applesColor.id = "apples";
    applesColor.color.a = 1.0;
    applesColor.color.r = 1.0;
    applesColor.color.g = 0.0;
    applesColor.color.b = 0.0;
    planning_scene_diff.object_colors.push_back(applesColor);
}

void spawnFloorInPlanningScene(moveit_msgs::msg::PlanningScene &planning_scene_diff) {
    moveit_msgs::msg::CollisionObject floorCollision;
    floorCollision.id = "floor";
    floorCollision.header.frame_id = "world";

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.5;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    floorCollision.primitive_poses.push_back(pose);

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 10.0;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 10.0;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 1.0;
    floorCollision.primitives.push_back(primitive);

    planning_scene_diff.world.collision_objects.push_back(floorCollision);

    moveit_msgs::msg::ObjectColor color;
    color.id = "floor";
    color.color.a = 1.0;
    color.color.r = 0.3;
    color.color.g = 0.2;
    color.color.b = 0.1;
    planning_scene_diff.object_colors.push_back(color);
}

moveit_msgs::msg::PlanningScene
createPlanningSceneDiff(const std::vector<DetachedTreeNode> &treeFlattened,
                        const std::vector<Eigen::Vector3d> &leafVertices,
                        const double appleRadius, const std::vector<Apple> &apples) {
    moveit_msgs::msg::PlanningScene planning_scene_diff;
    createTrunkInPlanningSceneMessage(treeFlattened, planning_scene_diff);
    spawnLeavesInPlanningScene(leafVertices, planning_scene_diff);
    spawnApplesInPlanningScene(appleRadius, apples, planning_scene_diff);
    spawnFloorInPlanningScene(planning_scene_diff);
    planning_scene_diff.is_diff = true;
    return planning_scene_diff;
}

AppleTreePlanningScene createMeshBasedAppleTreePlanningSceneMessage(const std::string &model_name) {

    const std::string cache_filename = "scene_cached_" + model_name + ".msg";
    std::stringstream prefix_stream;
    prefix_stream << "file://";
    prefix_stream << MYSOURCE_ROOT;
    prefix_stream << "/3d-models/";
    prefix_stream << model_name;
    std::string prefix = prefix_stream.str();

    auto cached_scene_info = read_ros_msg<moveit_msgs::msg::PlanningScene>(cache_filename);

    moveit_msgs::msg::PlanningScene planning_scene_message;

    if (!cached_scene_info) {

        std::cout << "Creating planning scene message for " << model_name << std::endl;

        planning_scene_message.is_diff = true;



                {
            const shape_msgs::msg::Mesh mesh = meshMsgFromResource(prefix + "_trunk.dae");

            const std::vector<shape_msgs::msg::Mesh> decomposition = convex_decomposition(mesh, 2.0);
            for (auto convex: decomposition | boost::adaptors::indexed(0)) {
                addColoredMeshCollisionShape(planning_scene_message, {0.5, 0.2, 0.1},
                                             "trunk" + std::to_string(convex.index()), convex.value());
            }
        }

        const shape_msgs::msg::Mesh apples = meshMsgFromResource(prefix + "_apples.dae");

        addColoredMeshCollisionShape(planning_scene_message, {1.0, 0.0, 0.0}, "apples", apples);

        addColoredMeshCollisionShape(
                planning_scene_message,
                {0.1, 0.7, 0.1},
                "leaves",
                meshMsgFromResource(prefix + "_leaves.dae"));

        planning_scene_message.name = model_name;

        save_ros_msg(cache_filename, planning_scene_message);

        std::cout << "Cached scene info for " << to_yaml(planning_scene_message) << std::endl;

    } else {
        std::cout << "Loaded cached scene info for " << model_name << std::endl;
        planning_scene_message = *cached_scene_info;
    }

    shape_msgs::msg::Mesh apples;
    for (const auto &collision_shape: planning_scene_message.world.collision_objects) {
        if (collision_shape.id == "apples") {
            apples = collision_shape.meshes[0];
            break;
        }
    }


    {
        auto enclosing = compute_enclosing_sphere(planning_scene_message, 0.1);

        std::cout << "center: " << enclosing.center << std::endl;
        std::cout << "radius: " << enclosing.radius << std::endl;

    }

    return {planning_scene_message, apples_from_connected_components(apples)};
}

