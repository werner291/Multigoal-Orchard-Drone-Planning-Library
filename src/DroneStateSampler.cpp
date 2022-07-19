#include "ompl_custom.h"
#include "DroneStateConstraintSampler.h"
#include "DroneStateSampler.h"

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

	// Allocate a MoveIt state to write the result to.
	moveit::core::RobotState out(space_->as<DroneStateSpace>()->getRobotModel());
	double *out_pos = out.getVariablePositions();

	// Sanity check to make sure we didn't add or remove any links in the robot.
	assert(out.getVariableCount() == 11);

	// Sample a point uniformly in a sphere, then add to the translation of the reference state.
	std::vector<double> translation_delta(3);
	rng_.uniformInBall(distance, translation_delta);
	// Also write it into the result variables.
	out_pos[0] = near_pos[0] + translation_delta[0];
	out_pos[1] = near_pos[1] + translation_delta[1];
	out_pos[2] = near_pos[2] + translation_delta[2];

	// Extract the rotation from the current set of variables.
	Eigen::Quaterniond current_rot(near_pos[6], near_pos[3], near_pos[4], near_pos[5]);
	// Add a random yaw-rotation to the reference rotation.
	Eigen::Quaterniond result_rot =
			current_rot * Eigen::AngleAxisd(rng_.uniformReal(-distance, distance), Eigen::Vector3d::UnitZ());
	// Write it into the result.
	out_pos[3] = result_rot.x();
	out_pos[4] = result_rot.y();
	out_pos[5] = result_rot.z();
	out_pos[6] = result_rot.w();

	// Add random angles to the arm joints.
	out_pos[7] = std::clamp(near_pos[7] + rng_.uniformReal(-distance, distance), -1.0, 1.0);
	out_pos[8] = std::clamp(near_pos[8] + rng_.uniformReal(-distance, distance), -1.0, 1.0);
	out_pos[9] = std::clamp(near_pos[8] + rng_.uniformReal(-distance, distance), -1.0, 1.0);
	out_pos[10] = near_pos[10] + rng_.uniformReal(-distance, distance);

	// Force-update
	out.update(true);

	// Convert to OMPL and write into the result variable.
	space_->as<DroneStateSpace>()->copyToOMPLState(state, out);

	// Enforce any relevant joint value bounds.
	space_->enforceBounds(state);
}

void DroneStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
	throw std::logic_error("Not implemented DroneStateSampler::sampleGaussian");
}