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

namespace mgodpl {

	struct EvaluationTrace {

		struct Frame {
			JointDistances joint_distances;
			std::vector<size_t> pts_seen;
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
								 const std::vector<std::vector<SurfacePoint>> &all_scannable_points,
								 const declarative::SensorScalarParameters &sensor_params,
								 const std::shared_ptr<const MeshOcclusionModel> &mesh_occlusion_model);

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
}

#endif //MGODPL_POINT_SCANNING_EVALUATION_H
