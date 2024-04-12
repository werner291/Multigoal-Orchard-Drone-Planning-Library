// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#ifndef MGODPL_PROBING_MOTIONS_H
#define MGODPL_PROBING_MOTIONS_H

#include <CGAL/AABB_tree.h>
#include "../math/Vec3.h"
#include "RobotModel.h"
#include "ApproachPath.h"
#include "RobotState.h"
#include "cgal_chull_shortest_paths.h"
#include "fcl_forward_declarations.h"
#include "../experiment_utils/TreeMeshes.h"
#include "RandomNumberGenerator.h"
#include <json/value.h>

namespace mgodpl {
	inline math::Vec3d arm_axis(const robot_model::RobotModel &robot, const robot_model::ForwardKinematicsResult &fk) {
		return fk.forLink(robot.findLinkByName("stick")).orientation.rotate(math::Vec3d::UnitY());
	}

	/**
	 * A method to find a path from a given target point out of the tree by moving along the axis of the arm.
	 * @return
	 */
	ApproachPath straightout(const robot_model::RobotModel &robot,
							 const RobotState &target_state,
							 const CGAL::AABB_tree<cgal::AABBTraits> &tree,
							 const cgal::Surface_mesh_shortest_path &algo);

	/**
	 * A method that combined straightout with uniform random sampling.
	 */
	std::optional<ApproachPath> uniform_straightout_approach(const mgodpl::math::Vec3d &target,
															 const mgodpl::robot_model::RobotModel &robot,
															 const fcl::CollisionObjectd &tree_trunk_object,
															 const cgal::CgalMeshData &mesh_data,
															 random_numbers::RandomNumberGenerator &rng,
															 size_t max_attempts,
															 double ee_distance = 0.0
	);

	class ApproachPlanner {

		virtual std::optional<ApproachPath> plan(
				const mgodpl::math::Vec3d &target,
				const mgodpl::robot_model::RobotModel &robot,
				const fcl::CollisionObjectd &tree_trunk_object,
				const cgal::CgalMeshData &mesh_data,
				random_numbers::RandomNumberGenerator &rng,
				double ee_distance = 0.0
				) const = 0;

		virtual Json::Value configuration() const = 0;

	};

	class StraightoutApproachPlanner : public ApproachPlanner {

		unsigned int goal_sample_attempts = 1000;

	public:

		[[nodiscard]] std::optional<ApproachPath> plan(const mgodpl::math::Vec3d &target,
													   const mgodpl::robot_model::RobotModel &robot,
													   const fcl::CollisionObjectd &tree_trunk_object,
													   const cgal::CgalMeshData &mesh_data,
													   random_numbers::RandomNumberGenerator &rng,
													   size_t max_attempts,
													   double ee_distance = 0.0) const {

			return uniform_straightout_approach(target,
												robot,
												tree_trunk_object,
												mesh_data,
												rng,
												max_attempts,
												ee_distance);

		}

		Json::Value configuration() const;

	};

	class GoalToGoalOptimizationMethod {

		/**
		 * Optimize a path to reduce its length without introducing collisions.
		 *
		 * @param robot 		The robot model.
		 * @param path 			The path to optimize.
		 * @param obstacle 		The obstacle to avoid.
		 * @return 				True if the path was optimized, false if it is unmodified.
		 */
		virtual bool optimize(const robot_model::RobotModel &robot,
							  RobotPath &path,
							  const fcl::CollisionObjectd &obstacle) const = 0;

	};

	class ShellPathPlanningMethod {

		/// Te approach planning method to use.
		const std::shared_ptr<const ApproachPlanner> approach_planner;

		/// The goal-to-goal optimization method to use, if any.
		/// This may be null, in which case no optimization is performed.
		const std::shared_ptr<const GoalToGoalOptimizationMethod> optimization_method;

	public:

		RobotPath plan_static(
				const robot_model::RobotModel &robot,
				const mgodpl::Mesh &trunk_mesh,
				const mgodpl::Mesh &leaves_mesh,
				const std::vector<math::Vec3d> &fruit_positions,
				const RobotState &initial_state
		);

		Json::Value configuration() {
			Json::Value config;
			return config;
		}

	};

}
#endif //MGODPL_PROBING_MOTIONS_H
