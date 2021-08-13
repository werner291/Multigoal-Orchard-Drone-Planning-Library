//
// Created by werner on 13-08-21.
//

#include "procedural_tree_generation.h"
#include <moveit/robot_state/conversions.h>
#include <Eigen/Geometry>
#include <random_numbers/random_numbers.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/util.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include "EndEffectorConstraintSampler.h"

EndEffectorPositionConstraintSampler::EndEffectorPositionConstraintSampler(
        const planning_scene::PlanningSceneConstPtr &scene, const std::string &groupName) :
        ConstraintSampler(scene, groupName) {}

bool EndEffectorPositionConstraintSampler::configure(const moveit_msgs::Constraints &constr) {

    // TODO Some defensive coding.
    ee_target_ = Eigen::Vector3d(
            constr.position_constraints[0].constraint_region.primitive_poses[0].position.x,
            constr.position_constraints[0].constraint_region.primitive_poses[0].position.y,
            constr.position_constraints[0].constraint_region.primitive_poses[0].position.z
    );

    ee_name_ = constr.position_constraints[0].link_name;

    radius_ = constr.position_constraints[0].constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];

    name_ = ee_name_ + "_position_with_mobile_base";

    return true;
}

bool EndEffectorPositionConstraintSampler::sample(moveit::core::RobotState &state,
                                                  const moveit::core::RobotState &reference_state,
                                                  unsigned int max_attempts) {

    for (int i = 0; i < max_attempts; i++) {
        state.setToRandomPositions();

        move_to_within(state, radius_ * (random_numbers::RandomNumberGenerator().uniformReal(0.0, 1.0 - std::numeric_limits<double>::epsilon())));

        if (scene_->isStateValid(state)) {
            return true;
        }
    }

    return false;
}

bool EndEffectorPositionConstraintSampler::project(moveit::core::RobotState &state, unsigned int max_attempts) {

    move_to_within(state, radius_);

    // No point in retrying, this method is deterministic.
    return scene_->isStateValid(state);
}

void EndEffectorPositionConstraintSampler::move_to_within(moveit::core::RobotState &state, double radius) const {
    Eigen::Vector3d ee_pos = state.getGlobalLinkTransform(ee_name_).translation();

    Eigen::Vector3d delta = ee_target_ - ee_pos;

    double norm = delta.norm();

    if (norm > radius) {
        delta *= ((norm - radius) / norm);

        double *positions = state.getVariablePositions();

        positions[0] += delta.x();
        positions[1] += delta.y();
        positions[2] += delta.z();

        state.update(true);
    }
}

const std::string &EndEffectorPositionConstraintSampler::getName() const {
    return name_;
}
