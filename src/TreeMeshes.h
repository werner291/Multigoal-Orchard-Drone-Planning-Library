
#ifndef NEW_PLANNERS_TREEMESHES_H
#define NEW_PLANNERS_TREEMESHES_H

#include <shape_msgs/msg/mesh.hpp>
#include <Eigen/Core>
#include <moveit/robot_state/robot_state.h>

struct TreeMeshes {
	std::string tree_name;
	shape_msgs::msg::Mesh leaves_mesh;
	shape_msgs::msg::Mesh trunk_mesh;
	std::vector<shape_msgs::msg::Mesh> fruit_meshes;
};

/**
 * Load the meshes for the given tree.
 *
 * @param treeName 		The name of the tree to load the meshes for.
 * @return 				The loaded meshes.
 */
TreeMeshes loadTreeMeshes(const std::string &treeName);

std::vector<std::string> getTreeModelNames(const std::string path = "./3d-models/");

struct SimplifiedOrchard {
	std::vector<std::pair<Eigen::Vector2d, TreeMeshes>> trees;
};

moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone);

std::vector<std::string> getTreeNames();

#endif //NEW_PLANNERS_TREEMESHES_H
