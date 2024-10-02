module;

// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <functional>
#include <optional>

#include "RobotModel.h"
#include "Mesh.h"
#include "RandomNumberGenerator.h"
#include "state_tools.h"

//
// Created by werner on 10/2/24.
//

export module sampling;

namespace mgodpl {

	/**
	 * @brief A structure to hold the horizontal and vertical radii for sampling.
	 *
	 * This structure contains two double values, h_radius and v_radius, which represent the horizontal and vertical radii respectively.
	 */
	struct SamplingRadii {
		double h_radius, v_radius;
	};

	/**
	 * @brief Calculates the sampling radii around a given mesh, with a margin.
	 *
	 * The margin represents the amount of free space around the mesh to sample from,
	 * as sampling is otherwise undefined in an infinite space.
	 *
	 * "Horizontal" is in the x/y plane, "vertical" is along the z-axis.
	 *
	 * @param meshs The Mesh object around which the sampling radii are to be calculated.
	 * @param sampler_margin The margin to be added to the calculated radii.
	 * @return A SamplingRadii object with the calculated horizontal and vertical radii.
	 */
	SamplingRadii sampling_radii_around_mesh(const Mesh &meshs, const double sampler_margin) {
		math::AABBd leaves_aabb = mesh_aabb(meshs);

		// Capture the maximum absolute value of any horizontal coordinate.
		double h_radius = std::max({leaves_aabb._max.x(),
									leaves_aabb._max.y(),
									std::abs(leaves_aabb._min.x()),
									std::abs(leaves_aabb._min.y())});

		double v_radius = leaves_aabb._max.z();

		return {
				h_radius + sampler_margin,
				v_radius + sampler_margin
		};
	}

	/**
	 * @brief Creates a uniform sampler function for a given robot model, random number generator, mesh, and margin.
	 *
	 * This function computes the sampling radii around the mesh using the provided margin and
	 * returns a lambda function that generates a uniform random state for the robot model.
	 *
	 * @param robot_model 	The RobotModel object for which the sampler function is to be created.
	 * @param rng 			The RandomNumberGenerator object to be used for generating random numbers.
	 * @param meshs 		The Mesh object around which the sampling radii are to be calculated.
	 * @param margin 		The margin to be added to the calculated radii.
	 * @return 				A function that generates a uniform random state for the robot model.
	 */
	export std::function<RobotState()> make_uniform_sampler_fn(const robot_model::RobotModel &robot_model,
															   random_numbers::RandomNumberGenerator &rng,
															   const Mesh &meshs,
															   const double margin) {

		auto [h_radius, v_radius] = sampling_radii_around_mesh(meshs, margin);

		return [&, h_radius, v_radius]() {
			return generateUniformRandomState(robot_model, rng, h_radius, v_radius, M_PI_2);
		};
	}
}