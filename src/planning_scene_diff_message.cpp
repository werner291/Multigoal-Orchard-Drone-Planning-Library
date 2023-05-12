#include <bullet/HACD/hacdHACD.h>
#include <rclcpp/serialization.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

#include "utilities/msgs_utilities.h"
#include "utilities/experiment_utils.h"

#include "planning_scene_diff_message.h"
#include "utilities/general_utilities.h"
#include "utilities/load_mesh.h"

moveit_msgs::msg::PlanningScene treeMeshesToMoveitSceneMsg(const TreeMeshes &tree_meshes, bool include_ground_plane) {

	const std::string cache_filename = "scene_cached_" + tree_meshes.tree_name + ".msg";
	std::stringstream prefix_stream;
	prefix_stream << "file://";
	prefix_stream << MYSOURCE_ROOT;
	prefix_stream << "/3d-models/";
	prefix_stream << tree_meshes.tree_name;
	std::string prefix = prefix_stream.str();

	auto cached_scene_info = read_ros_msg<moveit_msgs::msg::PlanningScene>(cache_filename);

	if (cached_scene_info) {
		std::cout << "Using cached scene info" << std::endl;
		return *cached_scene_info;
	}


	moveit_msgs::msg::PlanningScene planning_scene_message;

	planning_scene_message.name = tree_meshes.tree_name;

	planning_scene_message.is_diff = true;

	{
		const shape_msgs::msg::Mesh mesh = tree_meshes.trunk_mesh;

		const std::vector<shape_msgs::msg::Mesh> decomposition = convex_decomposition(mesh, 2.0);
		for (auto convex: decomposition | boost::adaptors::indexed(0)) {
			addColoredMeshCollisionShape(planning_scene_message, {0.5, 0.2, 0.1},
										 "trunk" + std::to_string(convex.index()),
										 convex.value());
		}
	}

	// Commented out because apples shouldn't be collision objects (for now, at least)
	//    for (const auto &[i, apple]: tree_meshes.fruit_meshes | boost::adaptors::indexed(0)) {
	//        addColoredMeshCollisionShape(planning_scene_message, {1.0, 0.0, 0.0}, "apple" + std::to_string(i), apple);
	//    }

	{
		const shape_msgs::msg::Mesh mesh = tree_meshes.leaves_mesh;
		addColoredMeshCollisionShape(planning_scene_message, {0.1, 0.7, 0.1}, "leaves", mesh);
	}

	// Make a ground plane.
	if (include_ground_plane) {
		moveit_msgs::msg::CollisionObject ground;
		ground.primitives.resize(1);
		ground.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
		ground.primitives[0].dimensions.resize(3);
		ground.primitives[0].dimensions = {50.0, 50.0, 50.0};
		ground.primitive_poses.resize(1);
		ground.primitive_poses[0].position.z = -25.0;
		ground.header.frame_id = "world";
		ground.id = "ground";
		planning_scene_message.world.collision_objects.push_back(std::move(ground));
	}

	save_ros_msg(cache_filename, planning_scene_message);

	std::cout << "Saved scene info for " << tree_meshes.tree_name << " to " << cache_filename << std::endl;

	return planning_scene_message;
}

AppleTreePlanningScene
createMeshBasedAppleTreePlanningSceneMessage(const std::string &model_name, bool include_ground_plane) {

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
                addColoredMeshCollisionShape(planning_scene_message,
                                             {0.5, 0.2, 0.1},
                                             "trunk" + std::to_string(convex.index()),
                                             convex.value());
            }
        }

        {
            const shape_msgs::msg::Mesh apples = meshMsgFromResource(prefix + "_fruit.dae");

            addColoredMeshCollisionShape(planning_scene_message, {1.0, 0.0, 0.0}, "apples", apples);
        }

        {
            addColoredMeshCollisionShape(planning_scene_message,
                                         {0.1, 0.7, 0.1},
                                         "leaves",
                                         meshMsgFromResource(prefix + "_leaves.dae"));

            planning_scene_message.name = model_name;
        }

        save_ros_msg(cache_filename, planning_scene_message);

		std::cout << "Saved scene info for " << model_name << " to " << cache_filename << std::endl;

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

    // Make a ground plane.
    if (include_ground_plane) {
        moveit_msgs::msg::CollisionObject ground;
        ground.primitives.resize(1);
		ground.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
		ground.primitives[0].dimensions = {50.0, 50.0, 50.0};
		ground.primitive_poses[0].position.z = -25.0;
        ground.header.frame_id = "world";
        ground.id = "ground";
        planning_scene_message.world.collision_objects.push_back(ground);
    }

    return {
		std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(planning_scene_message)),
		apples_from_connected_components(apples)
	};
}

std::vector<AppleTreePlanningScene> scenes_for_trees(const std::vector<std::string> &tree_names, int max_fruit) {

	// Rewrite using a for-loop.

	std::vector<AppleTreePlanningScene> scenes;

	for (const auto &tree_name: tree_names) {
		auto meshes = loadTreeMeshes(tree_name);

		// Skip huge trees, it crashes MoveIt.
		if (meshes.fruit_meshes.size() > max_fruit) {
			std::cout << "Skipping " << tree_name << " because it has " << meshes.fruit_meshes.size() << " apples"
					  << std::endl;
			continue;
		}

		scenes.push_back(AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(
				treeMeshesToMoveitSceneMsg(meshes)), .apples = meshes.fruit_meshes |
															   ranges::views::transform(appleFromMesh) |
															   ranges::to_vector});
	}

	return scenes;

}

