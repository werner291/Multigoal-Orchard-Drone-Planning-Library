
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

		std::vector<size_t> index_translation(mesh.vertices.size(), std::numeric_limits<size_t>::max());

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

				for (size_t i = 0; i < 3; ++i) {

					const auto& vertex = mesh.vertices[triangle.vertex_indices[i]];

					if (index_translation[triangle.vertex_indices[i]] == std::numeric_limits<size_t>::max()) {

						filtered_mesh.vertices.push_back(vertex);

						index_translation[triangle.vertex_indices[i]] = filtered_mesh.vertices.size() - 1;

					}

				}

				shape_msgs::msg::MeshTriangle filtered_triangle;
				filtered_triangle.vertex_indices[0] = index_translation[triangle.vertex_indices[0]];
				filtered_triangle.vertex_indices[1] = index_translation[triangle.vertex_indices[1]];
				filtered_triangle.vertex_indices[2] = index_translation[triangle.vertex_indices[2]];

				filtered_mesh.triangles.push_back(filtered_triangle);

			}

		}

		return filtered_mesh;

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
																			 const moveit_facade::JointSpacePoint &current_state,
																			 const std::vector<math::Vec3d>& newly_detected_fruits) {
		return {
				.filtered_tree_meshes = filterTreeMeshes(tree_meshes, grid_coords, seen_space),
				.grid_coords = grid_coords,
				.seen_space = seen_space,
				.occluded_space = occluded_space,
				.current_state = current_state,
				.newly_detected_fruits = newly_detected_fruits
		};
	}
}