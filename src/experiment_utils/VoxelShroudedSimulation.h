// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-11-23.
//

#ifndef MGODPL_VOXELSHROUDEDSIMULATION_H
#define MGODPL_VOXELSHROUDEDSIMULATION_H

#include "../math/Vec3.h"
#include "../planning/moveit_forward_declarations.h"
#include "TreeMeshes.h"
#include "../planning/JointSpacePoint.h"
#include "../planning/CollisionDetection.h"
#include "../visibility/GridVec.h"
#include "../math/AABBGrid.h"

namespace mgodpl {
	namespace planning {
		class BlindlyMoveToNextFruit;
	}
}

namespace mgodpl::simulation {


	/**
	 * A class that wraps the execution state of the simulation (without considering visualization).
	 */
	struct VoxelShroudedSimulation {

		// FIXME: Are we sure this is enough class members? (Sarcasm)

		/// The robot model.
		const moveit::core::RobotModelPtr robot_model;

		/// The tree model.
		const tree_meshes::TreeMeshes tree_model;

		const std::vector<math::Vec3d> fruit_positions;

		/// The algorithm.
		const std::shared_ptr<planning::BlindlyMoveToNextFruit> algorithm;

		/// The current state of the robot.
		moveit_facade::JointSpacePoint current_state;

		/// The total distance traveled by the robot.
		double total_distance = 0.0;

		/// The collision detection object (with global knowledge; do not leak to the algorithm).
		const moveit_facade::CollisionDetection collision_detection;

		/// The grid coordinates.
		const math::AABBGrid grid_coords;

		/// The grid of seen space.
		Grid3D<bool> seen_space;

		/// The grid of occluded space (corresponding to current_state)
		Grid3D<bool> occluded_space;

		/// The grid of apples in cells.
		Grid3D<std::vector<math::Vec3d>> apples_in_cells;

		std::optional<moveit_facade::JointSpacePoint> next_state{};

		const double step_size = 0.1;

		bool hasCollided = false;

		/// Constructor.
		VoxelShroudedSimulation(moveit::core::RobotModelPtr robot_model,
								tree_meshes::TreeMeshes tree_model,
								int seed,
								const double stepSize);

		void update();

		[[nodiscard]] bool is_done() const;

	};


}

#endif //MGODPL_VOXELSHROUDEDSIMULATION_H
