#include "procedural_tree_generation.h"
#include <moveit/robot_state/conversions.h>
#include <Eigen/Geometry>
#include <random_numbers/random_numbers.h>
#include "DroneStateConstraintSampler.h"


void moveEndEffectorToGoal(moveit::core::RobotState &state, double tolerance,
                                                        const Eigen::Vector3d &target) {
    double sample_radius = tolerance * (random_numbers::RandomNumberGenerator().uniformReal(0.0, 1.0 -
                                                                                                 std::numeric_limits<double>::epsilon()));

    Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

    Eigen::Vector3d delta = target - ee_pos;

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

void randomizeUprightWithBase(moveit::core::RobotState &state, double translation_bound) {
    state.setToRandomPositions();
    double *pos = state.getVariablePositions();

    random_numbers::RandomNumberGenerator rng;

    pos[0] = rng.uniformReal(-translation_bound, translation_bound);
    pos[1] = rng.uniformReal(-translation_bound, translation_bound);
    pos[2] = rng.uniformReal(0, translation_bound);

    Eigen::Quaterniond q(Eigen::AngleAxisd(rng.uniformReal(-M_PI, M_PI), Eigen::Vector3d::UnitZ()));
    pos[3] = q.x();
    pos[4] = q.y();
    pos[5] = q.z();
    pos[6] = q.w();

    state.update(true);
}
