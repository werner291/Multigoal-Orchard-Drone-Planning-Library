// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#ifndef MGODPL_PARAMETRIC_PATHS_H
#define MGODPL_PARAMETRIC_PATHS_H

#include <variant>
#include <json/value.h>
#include "scan_paths.h"
#include "LoadedTreeModel.h"

namespace mgodpl::declarative {

	/**
	 * A structure to hold the parameters related to a circular orbit.
	 */
	struct CircularOrbitParameters {
		double radius = 1.0; //< The radius of the orbit, in radii of the tree canopy.
		double inclination = 0.0; //< The inclination of the orbit, in radians.
		double ascendingNodeLongitude = 0.0; //< The longitude of the ascending node, in radians.
	};

	/**
	 * A structure describing a spherical oscillation orbit.
	 *
	 * See `mgodpl::latitude_oscillation_path` for more information about the parameters.
	 */
	struct SphericalOscillationParameters {
		double radius = 1.0; //< The radius of the oscillation, in radii of the tree canopy.
		double amplitude = 0.0; //< The amplitude of the oscillation, as a factor of PI/2 radius/
		unsigned int cycles = 1; //< The number of cycles of the oscillation.
	};

	/**
	 * Parameters relating to an outside-of-the-tree orbit path.
	 */
	struct OrbitPathParameters {
		const std::variant<CircularOrbitParameters, SphericalOscillationParameters> parameters; //< The parameters for the orbit path.
	};

	/**
	 * Instantiate a ParametricPath from an OrbitPathParameters.
	 *
	 * @param orbit 			The orbit parameters to use.
	 * @param tree_center 		The center of the tree that these paths are centered around.
	 * @param canopy_radius 	The radius of the tree's canopy.
	 * @return 					A ParametricPath that represents the given orbit with the given parameters.
	 */
	ParametricPath instantiatePath(const OrbitPathParameters &orbit,
								   const math::Vec3d &tree_center,
								   const double canopy_radius);

	ParametricPath instantiatePath(const OrbitPathParameters &orbit,
								   const experiments::LoadedTreeModel &treeModel);

	Json::Value toJson(const CircularOrbitParameters &orbitParameters);

	Json::Value toJson(const SphericalOscillationParameters &oscillationParameters);

	Json::Value toJson(const OrbitPathParameters &orbitPathParameters);

}

#endif //MGODPL_PARAMETRIC_PATHS_H
