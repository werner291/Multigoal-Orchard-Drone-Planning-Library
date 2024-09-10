// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#ifndef MGODPL_POINT_SCANNING_EVALUATION_H
#define MGODPL_POINT_SCANNING_EVALUATION_H

#include <json/value.h>
#include "../planning/RobotPath.h"
#include "surface_points.h"
#include "declarative/SensorModelParameters.h"
#include "MeshOcclusionModel.h"
#include "joint_distances.h"
#include "declarative_environment.h"

namespace mgodpl {

	struct EvaluationTrace {

		struct Frame {
			JointDistances joint_distances;
			std::vector<size_t> pts_seen;
			std::vector<size_t> interior_pts_seen;
		};

		std::vector<Frame> frames;
	};

	Json::Value toJson(const EvaluationTrace &trace);

	/**
	 * Evaluate a (static) path by simulating a robot moving along it and scanning the environment.
	 *
	 * At regular intervals, the robot is stopped and the number of scannable points that have been seen is recorded.
	 *
	 * @param path 							The path to evaluate.
	 * @param interpolation_speed 			The step size to use when moving along the path and evaluating the seen points.
	 * @param all_scannable_points 			The scannable points for each fruit.
	 * @param sensor_params 				The parameters for the sensor.
	 * @param mesh_occlusion_model 			The occlusion model for the mesh.
	 * @return 								A trace of the evaluation containing statistics for each frame.
	 */
	EvaluationTrace eval_static_path(const RobotPath &path,
									 double interpolation_speed,
									 const declarative::PointScanEvalParameters &params,
									 const declarative::PointScanEnvironment &env);

	/**
	 * Creates a seen/unseen status for each scannable point, initialized to false.
	 * @param all_scannable_points 		The scannable points for each fruit.
	 * @return 							A vector of vectors booleans, same structure as all_scannable_points, initialized to false.
	 */
	std::vector<std::vector<bool>> init_seen_status(const std::vector<std::vector<SurfacePoint>> &all_scannable_points);

	/**
	 * Set the seen/unseen status for each scannable point to true if it is visible from the current end effector position.
	 *
	 * Points that are already seen are not updated nor re-evaluated.
	 *
	 * @param sensor_params 				The scalar parameters for the sensor model.
	 * @param mesh_occlusion_model 			The datastructure to use to check for clear sightlines.
	 * @param eye_position 					The position of the sensor/eye.
	 * @param eye_forward 					The forward vector of the sensor/eye.
	 * @param all_scannable_points 			The scannable points for each fruit.
	 * @param ever_seen 					The seen/unseen status for each scannable point.
	 */
	void update_seen(const declarative::SensorScalarParameters &sensor_params,
					 const std::shared_ptr<const MeshOcclusionModel> &mesh_occlusion_model,
					 const math::Vec3d &eye_position,
					 const math::Vec3d &eye_forward,
					 const std::vector<std::vector<SurfacePoint>> &all_scannable_points,
					 std::vector<std::vector<bool>> &ever_seen);

	struct PointScanStats {
		std::vector<int> seen_per_fruit;
		int total_seen = 0;
	};

	/**
	 * Given a path and a vector of ScannablePoints, this function will count the number of points that have been seen
	 * in each cluster, as well as the total number of points seen.
	 *
	 * @param robot_model 		The robot model.
	 * @param path 				The path the robot has taken.
	 * @param scannable_points 	The scannable points for each fruit.
	 * @param step_size 		The step size for the path.
	 *
	 * @return The number of points seen in each cluster, and the total number of points seen.
	 */
	PointScanStats count_scanned_points(const mgodpl::robot_model::RobotModel robot_model,
										const RobotPath &path,
										const std::vector<ScannablePoints> &scannable_points,
										double step_size);

	/**
	 * @brief Computes the Axis-Aligned Bounding Box (AABB) for a given cluster of scannable points.
	 *
	 * The AABB is inflated to include the maximum scan distance in all directions.
	 *
	 * @param cluster A ScannablePoints object representing a cluster of points.
	 * @return The computed AABB for the given cluster.
	 */
	math::AABBd computeAABBForCluster(const ScannablePoints &cluster);

	/**
	 * @brief Computes the Axis-Aligned Bounding Boxes (AABBs) for a vector of clusters of scannable points.
	 *
	 * The AABBs are inflated to include the maximum scan distance in all directions.
	 *
	 * @param clusters A vector of ScannablePoints objects, each representing a cluster of points.
	 * @return A vector of computed AABBs for the given clusters.
	 */
	std::vector<math::AABBd> computeAABBsForClusters(const std::vector<ScannablePoints> &clusters);
}

#endif //MGODPL_POINT_SCANNING_EVALUATION_H
