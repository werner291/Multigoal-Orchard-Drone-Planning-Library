// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 28-6-23.
//

#include "default_occlusion_models.h"

const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 12> OCCLUSION_MODELS = {

		// Basic occlusion functions (For reasons that baffle me, we need to use make_pair for the first or the compiler is unable to deduce the type)
		std::make_pair("omniscient", [](TreeMeshes &meshes) -> CanSeeAppleFn {
			// If omniscient_occlusion doesn't need `meshes`, you can return it directly.
			return omniscient_occlusion;
		}),

		{"distance", [](TreeMeshes &meshes) {
			// Distance occlusion doesn't depend on `meshes`
			return distance_occlusion;
		}},

		// Field of view occlusions
		{"angle_end_effector", [](TreeMeshes &meshes) {
			// End effector's field of view occlusion
			return in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector");
		}},

		{"angle_base_link", [](TreeMeshes &meshes) {
			// Base link's field of view occlusion
			return in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link");
		}},

		// Mesh occlusions
		{"mesh_occlusion_end_effector", [](TreeMeshes &meshes) {
			// End effector's vision occlusion due to the mesh of leaves
			return mesh_occludes_vision(meshes.leaves_mesh, "end_effector");
		}},

		{"mesh_occlusion_base_link", [](TreeMeshes &meshes) {
			// Base link's vision occlusion due to the mesh of leaves
			return mesh_occludes_vision(meshes.leaves_mesh, "base_link");
		}},

		// Alpha shape occlusions
		{"alpha_occlusion_end_effector", [](TreeMeshes &meshes) {
			// End effector's vision occlusion due to the alpha shape approximation of leaves mesh
			return leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "end_effector");
		}},

		{"alpha_occlusion_base_link", [](TreeMeshes &meshes) {
			// Base link's vision occlusion due to the alpha shape approximation of leaves mesh
			return leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "base_link");
		}},

		// Combinations of mesh occlusion and angle occlusion
		{"mesh_and_angle_end_effector", [](TreeMeshes &meshes) {
			// Combined occlusion due to mesh of leaves and field of view of end effector
			return only_if_both(mesh_occludes_vision(meshes.leaves_mesh, "end_effector"),
								in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector"));
		}},

		{"mesh_and_angle_base_link", [](TreeMeshes &meshes) {
			// Combined occlusion due to mesh of leaves and field of view of base link
			return only_if_both(mesh_occludes_vision(meshes.leaves_mesh, "base_link"),
								in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link"));
		}},

		// Combinations of alpha shape occlusion and angle occlusion
		{"alpha_and_angle_end_effector", [](TreeMeshes &meshes) {
			// Combined occlusion due to alpha shape approximation of leaves mesh and field of view of end effector
			return only_if_both(leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "end_effector"),
								in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector"));
		}},

		{"alpha_and_angle_base_link", [](TreeMeshes &meshes) {
			// Combined occlusion due to alpha shape approximation of leaves mesh and field of view of base link
			return only_if_both(leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "base_link"),
								in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link"));
		}}
};
