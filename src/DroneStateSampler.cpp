//
// Created by werner on 24-10-21.
//

#include "UnionGoalSampleableRegion.h"
#include "ompl_custom.h"
#include <robowflex_library/builder.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/broadcaster.h>
#include "DroneStateConstraintSampler.h"
#include <fcl/fcl.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include "DroneStateSampler.h"

DroneStateSampler::DroneStateSampler(const ompl::base::StateSpace *space)
        : StateSampler(space) {}

void DroneStateSampler::sampleUniform(ompl::base::State *state) {
    moveit::core::RobotState st(space_->as<DroneStateSpace>()->getRobotModel());
    DroneStateConstraintSampler::randomizeUprightWithBase(st);
    space_->as<DroneStateSpace>()->copyToOMPLState(state, st);
}

void DroneStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) {

    moveit::core::RobotState nr(space_->as<DroneStateSpace>()->getRobotModel());
    space_->as<DroneStateSpace>()->copyToRobotState(nr, near);

    const double *near_pos = nr.getVariablePositions();

    moveit::core::RobotState out(space_->as<DroneStateSpace>()->getRobotModel());
    double *out_pos = out.getVariablePositions();

    assert(out.getVariableCount() == 11);

    out_pos[0] = near_pos[0] + distance;
    out_pos[1] = near_pos[1] + distance;
    out_pos[2] = near_pos[2] + distance;
//
    Eigen::Quaterniond current_rot(near_pos[6], near_pos[3], near_pos[4], near_pos[5]);
    Eigen::Quaterniond result_rot =
            current_rot * Eigen::AngleAxisd(rng_.uniformReal(-distance, distance), Eigen::Vector3d::UnitZ());
    out_pos[3] = result_rot.x();
    out_pos[4] = result_rot.y();
    out_pos[5] = result_rot.z();
    out_pos[6] = result_rot.w();
//
//    // TODO: Avoid hard-coding this stuff.
    out_pos[7] = std::clamp(near_pos[7] + rng_.uniformReal(-distance, distance), -1.0, 1.0);
    out_pos[8] = std::clamp(near_pos[8] + rng_.uniformReal(-distance, distance), -1.0, 1.0);
    out_pos[9] = std::clamp(near_pos[8] + rng_.uniformReal(-distance, distance), -1.0, 1.0);
    out_pos[10] = near_pos[10] + rng_.uniformReal(-distance, distance);
//
    out.update(true);
    space_->as<DroneStateSpace>()->copyToOMPLState(state, out);
    space_->enforceBounds(state);

}

void DroneStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
    ROS_ERROR("Not implemented DroneStateSampler::sampleGaussian");
}