// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file default_occlusion_models.h
 *
 * This file contains a list of default occlusion models, as a standardized reference list
 * to be easily accessible from a variety of experiments, should we wish to compare them.
 */

#ifndef MGODPL_DEFAULT_OCCLUSION_MODELS_H
#define MGODPL_DEFAULT_OCCLUSION_MODELS_H

#include <array>
#include <string>
#include "../../CanSeeApple.h"

/**
 * @brief A constant array of occlusion models associated with real-world tree leaves.
 *
 * Each model in the array is represented as a pair. The first element of the pair is a string representing the name of the occlusion model,
 * and the second element is a factory function that creates a `CanSeeAppleFn` function.
 *
 * The array contains the following occlusion models:
 *  - "omniscient": No occlusion is considered, the apple can always be seen irrespective of the tree leaves.
 *  - "distance": Occlusion is determined based on distance, not considering the tree leaves.
 *  - "angle_end_effector" and "angle_base_link": Occlusion is determined based on the angle from the "end_effector" and "base_link" respectively, not considering the tree leaves.
 *  - "mesh_occlusion_end_effector" and "mesh_occlusion_base_link": Occlusion is determined based on the vision occlusion by the tree leaves' mesh from the perspective of the "end_effector" and "base_link" respectively.
 *  - "alpha_occlusion_end_effector" and "alpha_occlusion_base_link": Occlusion is determined based on the vision occlusion by an alpha shape that approximates the tree leaves' mesh from the perspective of the "end_effector" and "base_link" respectively.
 *  - "mesh_and_angle_end_effector" and "mesh_and_angle_base_link": Occlusion is determined based on both mesh occlusion by the tree leaves and angle from the perspective of the "end_effector" and "base_link" respectively.
 *  - "alpha_and_angle_end_effector" and "alpha_and_angle_base_link": Occlusion is determined based on both alpha shape occlusion by the tree leaves and angle from the perspective of the "end_effector" and "base_link" respectively.
 *
 * The `CanSeeAppleFnFactory` is a function that takes a `TreeMeshes` object as input and returns a `CanSeeAppleFn` function.
 * The `CanSeeAppleFn` function determines whether an apple can be seen from a particular position considering the occlusion caused by the tree leaves as defined by the occlusion model.
 */
extern const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 12> OCCLUSION_MODELS;

#endif //MGODPL_DEFAULT_OCCLUSION_MODELS_H
