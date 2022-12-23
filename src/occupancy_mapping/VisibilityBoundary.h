// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_VISIBILITYBOUNDARY_H
#define NEW_PLANNERS_VISIBILITYBOUNDARY_H

#include <Eigen/Core>
#include <functional>

/**
 * BoundaryType is an enum that represents the type of boundary that a robot can encounter in its path planning.
 * OCCLUDING: A boundary that is not solid and can be passed through, but blocks the robot's sensors.
 * OBSTRUCTING: A solid boundary that the robot cannot pass through.
 */
enum BoundaryType {
	OCCLUDING, OBSTRUCTING
};

/**
  *  @struct BoundarySample
  *  A sample of a boundary surface bounding observable by the robot at a specific point in time.
  *  @var surface_point
  *  The surface point at which this boundary exists.
  *  @var boundary_type
  *  The type of boundary: whether it merely occludes, or also obstructs.
  */
struct BoundarySample {
	/// The surface point at which this boundary exists.
	Eigen::Vector3d surface_point;
	/// The type of boundary: whether it merely occludes, or also obstructs.
	BoundaryType boundary_type;
};

/**
  * @using RegionDefinitionFn
  * A function that, given a point in space, returns the closest point on any of the boundary surfaces that currently limit the robot's vision. The bounding surfaces are assumed to be star-shaped, centered on the robot's sensor center.
  */
using RegionDefinitionFn = std::function<BoundarySample(const Eigen::Vector3d &)>;

#endif //NEW_PLANNERS_VISIBILITYBOUNDARY_H
