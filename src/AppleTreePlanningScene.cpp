#include <bullet/HACD/hacdHACD.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>

#include "utilities/msgs_utilities.h"
#include "utilities/experiment_utils.h"

#include "AppleTreePlanningScene.h"
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

	if (!cached_scene_info) {

		std::cout << "Cached scene info not found, generating new scene info" << std::endl;

		moveit_msgs::msg::PlanningScene planning_scene_message;

		planning_scene_message.name = tree_meshes.tree_name;

		planning_scene_message.is_diff = true;

		{
			const shape_msgs::msg::Mesh mesh = tree_meshes.trunk_mesh;

			const std::vector<shape_msgs::msg::Mesh> decomposition = convex_decomposition(mesh, 2.0);
			for (auto convex: decomposition | boost::adaptors::indexed(0)) {
				addColoredMeshCollisionShape(planning_scene_message,
											 {0.5, 0.2, 0.1},
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
			ground.primitives[0].dimensions[0] = 50.0;
			ground.primitives[0].dimensions[1] = 50.0;
			ground.primitives[0].dimensions[2] = 50.0;
			ground.primitive_poses.resize(1);
			ground.primitive_poses[0].position.z = -25.0;
			ground.header.frame_id = "world";
			ground.id = "ground";
			planning_scene_message.world.collision_objects.push_back(std::move(ground));
		}

		save_ros_msg(cache_filename, planning_scene_message);

		std::cout << "Saved scene info for " << tree_meshes.tree_name << " to " << cache_filename << std::endl;

		return planning_scene_message;
	} else {
		std::cout << "Loaded cached scene info for " << tree_meshes.tree_name << std::endl;
		return *cached_scene_info;
	}
}

AppleTreePlanningScene
createMeshBasedAppleTreePlanningSceneMessage(const std::string &model_name, bool include_ground_plane) {

	auto tree_meshes = loadTreeMeshes(model_name);

    return {
		std::make_shared<moveit_msgs::msg::PlanningScene>(treeMeshesToMoveitSceneMsg(tree_meshes, include_ground_plane)),
		tree_meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector
	};
}

std::vector<AppleTreePlanningScene> scenes_for_trees(const std::vector<std::string> &tree_names, const size_t max_fruit) {
	std::vector<AppleTreePlanningScene> result;

	for (const auto &tree_name : tree_names) {
		auto meshes = loadTreeMeshes(tree_name);

		if (meshes.fruit_meshes.size() > max_fruit) {
			std::cout << "Skipping " << tree_name << " because it has " << meshes.fruit_meshes.size() << " fruit" << std::endl;
			continue;
		}

		auto scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(treeMeshesToMoveitSceneMsg(meshes)));

		std::vector<Apple> apples;
		for (const auto &fruit_mesh : meshes.fruit_meshes) {
			apples.push_back(appleFromMesh(fruit_mesh));
		}

		AppleTreePlanningScene scene{.scene_msg = scene_msg, .apples = apples};

		result.push_back(std::move(scene));
	}

	return result;

}

AppleTreePlanningScene createSceneFromTreeModels(const TreeMeshes &tree_models) {
	std::vector<Apple> apples;

	for (const auto &mesh : tree_models.fruit_meshes) {
		apples.push_back(appleFromMesh(mesh));
	}

	AppleTreePlanningScene scene {
			.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(treeMeshesToMoveitSceneMsg(tree_models, false))),
			.apples = apples
	};

	return scene;
}

AppleTreePlanningScene createSceneFromSimplifiedOrchard(const SimplifiedOrchard &orchard) {

	AppleTreePlanningScene scene;
	scene.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>();

	for (auto tree : orchard.trees) {

		// We do this first so we can benefit from caching.
		AppleTreePlanningScene tree_scene = createSceneFromTreeModels(tree.second);

		// Now, we copy the collision objects from the tree scene into the orchard scene.
		for (auto &collision_object: tree_scene.scene_msg->world.collision_objects) {
			collision_object.pose.position.x += tree.first.x();
			collision_object.pose.position.y += tree.first.y();
			scene.scene_msg->world.collision_objects.push_back(collision_object);
		}

		for (auto apple: tree_scene.apples) {
			apple.center.x() += tree.first.x();
			apple.center.y() += tree.first.y();
			scene.apples.push_back(apple);
		}

	}

	return scene;

}
