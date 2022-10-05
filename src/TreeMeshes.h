
#ifndef NEW_PLANNERS_TREEMESHES_H
#define NEW_PLANNERS_TREEMESHES_H

#include <shape_msgs/msg/mesh.hpp>
#include <Eigen/Core>
#include "utilities/load_mesh.h"

struct TreeMeshes {
	shape_msgs::msg::Mesh leaves_mesh;
	shape_msgs::msg::Mesh trunk_mesh;
	shape_msgs::msg::Mesh fruit_mesh;
};

/**
 * Load the meshes for the given tree.
 *
 * @param treeName 		The name of the tree to load the meshes for.
 * @return 				The loaded meshes.
 */
TreeMeshes loadTreeMeshes(const std::string& treeName);

struct SimplifiedOrchard {
	std::vector<std::pair<Eigen::Vector2d, TreeMeshes>> trees;
};

#endif //NEW_PLANNERS_TREEMESHES_H
