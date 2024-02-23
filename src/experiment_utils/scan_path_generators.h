// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9-2-24.
//

#ifndef SCAN_PATH_GENERATORS_H
#define SCAN_PATH_GENERATORS_H

#include "JsonMeta.h"
#include "scan_paths.h"

namespace mgodpl
{
    /**
     * @brief Generates a variety of orbits for a given center and target radius.
     *
     * This function generates a variety of orbits for a given center and target radius.
     * The orbits are represented as a vector of pairs, where each pair consists of a string
     * describing the type of the orbit and a vector of JsonMeta objects representing the orbits.
     * Each JsonMeta object pairs a ParametricPath object representing the orbit with a Json::Value
     * object storing the metadata of the orbit.
     *
     * @param fruit_center The center of the orbits.
     * @param target_radius The target radius of the orbits.
     * @return A vector of pairs representing the generated orbits and their types.
     */
    std::vector<std::pair<std::string, std::vector<JsonMeta<mgodpl::ParametricPath>>>> gen_orbits(
        const math::Vec3d &fruit_center,
        const double target_radius);

	/**
	 * @brief This function generates a list of orbits based on the input parameters and allows the user to select the type of orbit.
	 *
	 * The function first generates a list of orbits using the gen_orbits function. It then prompts the user to select the type of orbit.
	 * The user can select a specific type of orbit or choose to select all orbits. If the user selects a valid orbit type, the function will return a vector of JsonMeta objects representing
	 * the selected orbits.
	 *
	 * @param fruit_center The center of the orbits.
	 * @param EYE_ORBIT_RADIUS The radius of the orbits.
	 * @return A vector of JsonMeta objects representing the selected orbits.
	 */
	std::vector<JsonMeta<ParametricPath>> getOrbits(const mgodpl::math::Vec3d& fruit_center, double EYE_ORBIT_RADIUS);

	/**
	 * @brief Generates a single orbit based on user input.
	 *
	 * This function prompts the user to select an orbit type and then generates the corresponding orbit.
	 * The user is also prompted to enter the necessary parameters for the selected orbit type.
	 *
	 * The radii and height are multiplied by the base radius to generate the orbit.
	 *
	 * @param fruit_center The center of the orbit.
	 * @param base_radius The base radius for the orbit.
	 * @return The generated orbit.
	 */
	ParametricPath getSingleOrbit(const math::Vec3d &fruit_center, double base_radius);
}

#endif //SCAN_PATH_GENERATORS_H