// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_INFORMATIONGAINMODEL_H
#define NEW_PLANNERS_INFORMATIONGAINMODEL_H

#include <vector>
#include <Eigen/Geometry>

/**
 * Abstract base class for some method of scoring the potential gain in information
 * by a sensor by placing it in either a single pose or multiple poses.
 */
class InformationGainModel {

	/**
	 * Score the potential gain in information by a sensor if it
	 * were to be placed in a set of poses.
	 *
	 * The method will return a vector of scores, one for each pose,
	 * to model the idea of diminishing returns as the sensor is put
	 * in more and more poses.
	 *
	 * @param poses 		The poses to score.
	 * @return 				The score after each pose.
	 */
	virtual std::vector<double> evaluate_path(const std::vector<Eigen::Isometry3d> &poses) const = 0;

	virtual double evaluate_single(const Eigen::Isometry3d &pose) const = 0;

};


#endif //NEW_PLANNERS_INFORMATIONGAINMODEL_H
