
#include "VoxelShroudedSceneInfo.h"

#include "TreeMeshes.h"
#include "../math/AABBGrid.h"
#include "../visibility/GridVec.h"
#include "../math/Triangle.h"
#include "../math/aabb_of.h"

namespace mgodpl::experiments {

	using namespace math;
	using namespace tree_meshes;

	shape_msgs::msg::Mesh filterMesh(const shape_msgs::msg::Mesh& mesh,
									 const AABBGrid& grid_coords,
									 const Grid3D<bool>& seen_space) {

		shape_msgs::msg::Mesh filtered_mesh;

		for (const auto& triangle : mesh.triangles) {

			const auto& v0 = mesh.vertices[triangle.vertex_indices[0]];
			const auto& v1 = mesh.vertices[triangle.vertex_indices[1]];
			const auto& v2 = mesh.vertices[triangle.vertex_indices[2]];

			const auto& aabb = math::aabb_of(math::Triangle {
				Vec3d(v0.x, v0.y, v0.z),
				Vec3d(v1.x, v1.y, v1.z),
				Vec3d(v2.x, v2.y, v2.z)
			});

			const auto& centroid = aabb.center();

			const auto& grid_coord = grid_coords.getGridCoordinates(centroid);

			if (grid_coord.has_value() && seen_space[*grid_coord]) {
				filtered_mesh.triangles.push_back(triangle);
			}

		}

	}

	TreeMeshes filterTreeMeshes(const TreeMeshes &tree_meshes,
								const AABBGrid &grid_coords,
								const Grid3D<bool> &seen_space) {

		return {
				.tree_name = tree_meshes.tree_name + "_filtered",
				.leaves_mesh = filterMesh(tree_meshes.leaves_mesh, grid_coords, seen_space),
				.trunk_mesh = filterMesh(tree_meshes.trunk_mesh, grid_coords, seen_space),
				.fruit_meshes = {} // Don't care about fruits for now.
		};

	}

	VoxelShroudedSceneInfoUpdate VoxelShroudedSceneInfoUpdate::make_filtered(const TreeMeshes &tree_meshes,
																			 const AABBGrid &grid_coords,
																			 const Grid3D<bool> &seen_space,
																			 const Grid3D<bool> &occluded_space,
																			 const moveit_facade::JointSpacePoint &current_state) {
		return {
				.filtered_tree_meshes = filterTreeMeshes(tree_meshes, grid_coords, seen_space),
				.grid_coords = grid_coords,
				.seen_space = seen_space,
				.occluded_space = occluded_space,
				.current_state = current_state
		};
	}
}