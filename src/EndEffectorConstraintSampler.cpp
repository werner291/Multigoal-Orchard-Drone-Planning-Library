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

DroneStateConstraintSampler::DroneStateConstraintSampler(
        const planning_scene::PlanningSceneConstPtr &scene, const std::string &groupName) :
        ConstraintSampler(scene, groupName) {}

bool DroneStateConstraintSampler::configure(const moveit_msgs::Constraints &constr) {

    if (constr.position_constraints.empty()) {
        ee_target_ = {};
    } else {
        ee_target_ = {{
            .target = Eigen::Vector3d(
                    constr.position_constraints[0].constraint_region.primitive_poses[0].position.x,
                    constr.position_constraints[0].constraint_region.primitive_poses[0].position.y,
                    constr.position_constraints[0].constraint_region.primitive_poses[0].position.z),
                    .radius = constr.position_constraints[0].constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]
        }};
    }

    name_ = ee_name_ + "_position_with_mobile_base";

    return true;
}

bool DroneStateConstraintSampler::sample(moveit::core::RobotState &state,
                                         const moveit::core::RobotState &reference_state,
                                         unsigned int max_attempts) {

    for (int i = 0; i < max_attempts; i++) {

        state.setToRandomPositions();
        double* pos = state.getVariablePositions();

        random_numbers::RandomNumberGenerator rng;

        pos[0] = rng.uniformReal(-20.0,20.0);
        pos[1] = rng.uniformReal(-20.0,20.0);
        pos[1] = rng.uniformReal(0,20.0);

        Eigen::Quaterniond q(Eigen::AngleAxisd( rng.uniformReal(-M_PI, M_PI), Eigen::Vector3d::UnitZ() ));
        pos[3] = q.x();
        pos[4] = q.y();
        pos[5] = q.z();
        pos[6] = q.w();

        state.update(true);

        if (ee_target_) {
            double sample_radius = ee_target_->radius * (random_numbers::RandomNumberGenerator().uniformReal(0.0, 1.0 - std::numeric_limits<double>::epsilon()));

            Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

            Eigen::Vector3d delta = ee_target_->target - ee_pos;

            double norm = delta.norm();

            if (norm > sample_radius) {
                delta *= ((norm - sample_radius) / norm);

                double *positions = state.getVariablePositions();

                positions[0] += delta.x();
                positions[1] += delta.y();
                positions[2] += delta.z();

                state.update(true);
            }

        }

        if (scene_->isStateValid(state)) {
            return true;
        }
    }

    return false;
}

bool DroneStateConstraintSampler::project(moveit::core::RobotState &state, unsigned int max_attempts) {
    return false; //TODO
}

const std::string &DroneStateConstraintSampler::getName() const {
    return name_;
}
