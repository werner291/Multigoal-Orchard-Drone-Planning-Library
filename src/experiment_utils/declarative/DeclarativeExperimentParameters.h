// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/8/24.
//

#ifndef MGODPL_DECLARATIVEEXPERIMENTPARAMETERS_H
#define MGODPL_DECLARATIVEEXPERIMENTPARAMETERS_H

#include <json/value.h>
#include "../scan_paths.h"

/**
 * @struct SensorScalarParameters
 * @brief A structure to hold the scalar parameters related to a sensor.
 */
struct SensorScalarParameters {
	double maxViewDistance; //< The maximum distance from the sensor a point can be to be considered visible.
	double minViewDistance; //< The minimum distance from the sensor a point can be to be considered visible.
	double fieldOfViewAngle; //< The field of view angle of the sensor (from the center to the edge).
	double maxScanAngle; //< The maximum angle a point can be from the forward direction of the sensor to be considered visible.
};

Json::Value toJson(const SensorScalarParameters &sensorParameters);

/// Use the original set of fruit.
struct Unchanged {
};

/// Select a random subset of the fruit.
struct RandomSubset {
	int count; //< The number of elements to select from the set.
};

/// Replace the set of fruit with a new set.
struct Replace {
	int count; //< The number of elements to replace in the set.
};

/**
 * @struct TreeModelParameters
 *
 * @brief A structure to hold the parameters related to a tree model in an experiment.
 */
struct TreeModelParameters {
	const std::string name; //< A model name; corresponds to one of the models in the `3d-models` directory.
	const double leaf_scale; //< The scaling factor for the leaves; 1.0 skips the rescaling.
	const std::variant<Unchanged, RandomSubset, Replace> fruit_subset; //< The subset of fruits to use in the experiment.
	const int seed; //< The seed for randomizing the tree model.
};

Json::Value toJson(const TreeModelParameters &treeModelParameters);

#endif //MGODPL_DECLARATIVEEXPERIMENTPARAMETERS_H
