#include "ompl_custom.h"
#include "DroneStateConstraintSampler.h"
#include "DroneStateSampler.h"
#include "utilities/moveit.h"

DroneStateSampler::DroneStateSampler(const ompl::base::StateSpace *space, double translationBound)
		: StateSampler(space), translation_bound(translationBound) {
}

void DroneStateSampler::sampleUniform(ompl::base::State *state) {
	// Allocate a Moveit state
	moveit::core::RobotState st(space_->as<DroneStateSpace>()->getRobotModel());

	// Sample uniformly.
	randomizeUprightWithBase(st, translation_bound);

	// Convert and write to the state.
	space_->as<DroneStateSpace>()->copyToOMPLState(state, st);
}

void DroneStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) {

	// Allocate a Moveit state and convert from OMPL.
	moveit::core::RobotState nr(space_->as<DroneStateSpace>()->getRobotModel());
	space_->as<DroneStateSpace>()->copyToRobotState(nr, near);
	const double *near_pos = nr.getVariablePositions();

	// Sanity check to make sure we didn't add or remove any links in the robot.
	assert(nr.getVariableCount() == 11);

	auto out = sampleStateNearByUpright(nr, distance);

	// Convert to OMPL and write into the result variable.
	space_->as<DroneStateSpace>()->copyToOMPLState(state, out);

	// Enforce any relevant joint value bounds.
	space_->enforceBounds(state);
}

void DroneStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
	throw std::logic_error("Not implemented DroneStateSampler::sampleGaussian");
}