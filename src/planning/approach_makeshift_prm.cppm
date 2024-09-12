// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/12/24.
//

module;

#include "RobotState.h"
#include "RandomNumberGenerator.h"

export module approach_makeshift_prm;

export namespace mgodpl {

	/**
	 * Sample a RobotState nearby the original state.
	 *
	 * @param state 	The original state.
	 * @param rng 			A random number generator.
	 * @param noise_scale 	The scale of the noise.
	 */
	RobotState sample_nearby_state(RobotState state,
								   random_numbers::RandomNumberGenerator &rng,
								   double distance) {

		// Generate a random direction:
		math::Vec3d translation_noise_vector(rng.gaussian01(),
											  rng.gaussian01(),
											  rng.gaussian01());
		translation_noise_vector.normalize();
		// Scale it by the distance:
		translation_noise_vector *= distance * rng.uniform01();

		// Apply the noise:
		state.base_tf.translation += translation_noise_vector;

		// Convert the scalar rotation to a Quaternion rotation around the Z-axis:
		math::Quaterniond orientation_noise = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), rng.uniformReal(-distance, distance));
		// Apply the noise:
		state.base_tf.orientation = state.base_tf.orientation * orientation_noise;

		// Apply the joint nois:
		for (double & joint_value : state.joint_values) {
			joint_value += rng.uniformReal(-distance, distance);
			// Apply limit:
			joint_value = std::clamp(joint_value, -M_PI_2, M_PI_2);
		}

		return state;

	}

	/**
	 * @brief Generates a RobotState that is a random sample along the motion from a start state to a goal state.
	 * The sample is generated in two steps:
	 * 1. A state is randomly interpolated between the start and goal states.
	 * 2. A state is sampled within a small neighborhood of the interpolated state, where the distance is exponentially distributed.
	 *
	 * @param start The starting RobotState.
	 * @param goal The goal RobotState.
	 * @param rng A random number generator.
	 * @param gaussian_scale A scale factor for the Gaussian distribution used to generate the distance for the second step.
	 * @return A RobotState that is a random sample along the motion from the start state to the goal state.
	 */
	RobotState makeshift_exponential_sample_along_motion(const RobotState &start,
														 const RobotState &goal,
														 random_numbers::RandomNumberGenerator &rng,
														 double gaussian_scale) {

		// Step 1: grab a state randomly interpolated between start and goal.
		RobotState sample = interpolate(start, goal, rng.uniform01());

		// Step 2: sample a state within a small neighborhood of the sample, where the distance is exponentially distributed.
		double distance = std::abs(rng.gaussian01()) * gaussian_scale;

		return sample_nearby_state(sample, rng, distance);

	}
}