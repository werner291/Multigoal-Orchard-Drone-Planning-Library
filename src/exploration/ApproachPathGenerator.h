// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 25-11-22.
//

#ifndef NEW_PLANNERS_APPROACHPATHGENERATOR_H
#define NEW_PLANNERS_APPROACHPATHGENERATOR_H

#include <moveit/robot_model/robot_model.h>
#include <Eigen/Core>
#include <utility>
#include "DirectPointCloudCollisionDetection.h"
#include "DynamicMeshHullAlgorithm.h"
#include "../shell_space/CGALMeshShell.h"
#include "VersionedRevalidableKeyValueCache.h"


/**
 * A utility class that computes approach paths. It, by choice, does not use OMPL as I'd like to try
 * to sample whole paths at once, instead of sampling one state at a time.
 *
 * @tparam ShellPoint 		The type of points used in the shell.
 */
template<typename ShellPoint>
class ApproachPaths {

	struct EigenHash {

		std::size_t operator()(const Eigen::Vector3d &v) const {
			return (std::hash<double>()(v.x()) * 31 + std::hash<double>()(v.y())) * 31 + std::hash<double>()(v.z());
		}

	};

	/// Cached approach paths for past requests.
	VersionedRevalidableKeyValueCache<Eigen::Vector3d, std::optional<RobotPath>, size_t, EigenHash> approachPaths;

	/**
	 * Compute an approach path for a given target point, not using any cached data.
	 *
	 * @param target_point 				The point to approach.
	 * @param collision_detector 		The collision detector to use.
	 * @param workspace_shell 			The workspace shell to use. (TODO: redundant with shell_space)
	 * @param shell_space 				The shell space to use.
	 * @return 							The approach path, or std::nullopt if no path could be found.
	 */
	std::optional<RobotPath> computeApproachPath(Eigen::Vector3d target_point,
												 const DirectPointCloudCollisionDetection &collision_detector,
												 const WorkspaceShell<ShellPoint> &workspace_shell,
												 const MoveItShellSpace<ShellPoint> &shell_space) const {

		auto shell_state = shell_space.stateFromPoint(workspace_shell.nearest_point_on_shell(target_point));

		// Create a path generator that will generate possible paths that need to be checked for collisions
		CandidatePathGenerator gen(shell_state, target_point);

		// Generate one candidate path (TODO: generate more than one)
		auto candidate = gen.generateCandidatePath();

		// Check for collisions
		if (collision_detector.checkCollisionInterpolated(candidate, COLLISION_DETECTION_MAX_STEP)) {
			// The candidate path is in collision; we conclude the target is unreachable.
			return std::nullopt;
		} else {
			// The candidate path is valid, so we can use it.
			return {candidate};
		}
	}

public:

	/**
	 * Compute an approach path for a given target point, or return one from the cache if possible.
	 *
	 * @param target_point 				The point to approach.
	 * @param collision_detector 		The collision detector to use.
	 * @param workspace_shell 			The workspace shell to use. (TODO: redundant with shell_space)
	 * @param shell_space 				The shell space to use.
	 * @return 							The approach path, or std::nullopt if no path could be found.
	 */
	std::optional<RobotPath> approachPathForTarget(Eigen::Vector3d target_point,
												   const DirectPointCloudCollisionDetection &collision_detector,
												   const WorkspaceShell<ShellPoint> &workspace_shell,
												   const MoveItShellSpace<ShellPoint> &shell_space) {

		auto revalidate = [&](const Eigen::Vector3d &target, const std::optional<RobotPath> &old_path) {
			return !old_path || collision_detector.checkCollisionInterpolated(*old_path, COLLISION_DETECTION_MAX_STEP);
		};

		auto compute = [&](const Eigen::Vector3d &target) -> std::optional<RobotPath> {
			return computeApproachPath(target_point, collision_detector, workspace_shell, shell_space);
		};

		return approachPaths.get_revalidate_or_compute(target_point,
													   collision_detector.getVersion(),
													   revalidate,
													   compute);

	}

	void invalidate(const Eigen::Vector3d &target_point) {
		approachPaths.invalidate(target_point);
	}

};

#endif //NEW_PLANNERS_APPROACHPATHGENERATOR_H
