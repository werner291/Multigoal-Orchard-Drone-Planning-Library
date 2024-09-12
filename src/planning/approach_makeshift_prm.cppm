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
	 * @param rng 		A random number generator.
	 * @param distance 	The maximum distance from the original state.
	 */
	RobotState sample_nearby_state(RobotState state,
								   random_numbers::RandomNumberGenerator &rng,
								   double distance) {

		// Generate noise:
		double translation_noise_scalar = rng.uniform01();
		double base_rotation_noise_scalar = rng.uniform01();
		std::vector<double> joint_noise(state.joint_values.size());
		for (double &joint_value : joint_noise) {
			joint_value = rng.uniform01();
		}

		// Add it all up:
		double noise_total = translation_noise_scalar + base_rotation_noise_scalar + std::accumulate(joint_noise.begin(), joint_noise.end(), 0.0);

		// Divide by the distance such that the noise is proportional to the distance:
		double divider = noise_total / distance;

		// Normalize the noise such that it doesn't exceed the distance:
		translation_noise_scalar /= noise_total;
		base_rotation_noise_scalar /= noise_total;
		for (double &joint_value : joint_noise) {
			joint_value /= noise_total;
		}

		// Sanity check: the noise should not exceed the distance.
		assert(translation_noise_scalar + base_rotation_noise_scalar + std::accumulate(joint_noise.begin(), joint_noise.end(), 0.0) < distance);

		// Generate a random direction:
		math::Vec3d translation_noise_vector(rng.gaussian01(),
											  rng.gaussian01(),
											  rng.gaussian01());
		translation_noise_vector.normalize();
		// Scale it by the distance:
		translation_noise_vector *= translation_noise_scalar;

		// Apply the noise:
		state.base_tf.translation += translation_noise_vector;

		// Convert the scalar rotation to a Quaternion rotation around the Z-axis:
		math::Quaterniond orientation_noise = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), base_rotation_noise_scalar);
		// Apply the noise:
		state.base_tf.orientation = state.base_tf.orientation * orientation_noise;

		// Apply the joint noise:
		for (size_t i = 0; i < state.joint_values.size(); ++i) {
			state.joint_values[i] += joint_noise[i];
			// Apply limit:
			state.joint_values[i] = std::clamp(state.joint_values[i], -M_PI_2, M_PI_2);
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