
#pragma once

#include "TreeMeshes.h"

namespace mgodpl {

	template<typename T>
	class Grid3D;

	namespace math {
		class AABBGrid;
	}

	namespace moveit_facade {
		struct JointSpacePoint;
	}

	namespace experiments {

		tree_meshes::TreeMeshes filterTreeMeshes(const tree_meshes::TreeMeshes &tree_meshes,
												 const math::AABBGrid &grid_coords,
												 const Grid3D<bool> &seen_space);

		struct VoxelShroudedSceneInfoUpdate {

			const tree_meshes::TreeMeshes filtered_tree_meshes;
			const math::AABBGrid &grid_coords;
			const Grid3D<bool> &seen_space;
			const Grid3D<bool> &occluded_space;
			const moveit_facade::JointSpacePoint &current_state;
			const std::vector<math::Vec3d> newly_detected_fruits;

			static VoxelShroudedSceneInfoUpdate make_filtered(
					const tree_meshes::TreeMeshes &tree_meshes,
					const math::AABBGrid &grid_coords,
					const Grid3D<bool> &seen_space,
					const Grid3D<bool> &occluded_space,
					const moveit_facade::JointSpacePoint &current_state,
					const std::vector<math::Vec3d>& newly_detected_fruits
					);
		};



	}
}