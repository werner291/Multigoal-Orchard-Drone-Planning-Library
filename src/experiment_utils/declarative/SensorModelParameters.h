// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/8/24.
//

#ifndef MGODPL_SENSORMODELPARAMETERS_H
#define MGODPL_SENSORMODELPARAMETERS_H

#include <json/value.h>
#include "../scan_paths.h"

namespace mgodpl::declarative
{
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
}

#endif //MGODPL_SENSORMODELPARAMETERS_H
