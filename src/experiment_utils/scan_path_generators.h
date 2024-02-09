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
}

#endif //SCAN_PATH_GENERATORS_H