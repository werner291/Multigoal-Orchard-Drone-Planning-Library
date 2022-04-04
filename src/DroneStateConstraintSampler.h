#ifndef NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H
#define NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H

#include <random_numbers/random_numbers.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_allocator.h>

struct SphericalConstraint {
    Eigen::Vector3d target;
    double radius;
};

/**
 *
 * A constraint sampler that samples states for the drone,
 * optionally with the end-effector within a sphere for the purpose of goal sampling.
 *
 * When the goal sphere is specified, samples are taken by picking a point uniformly within the sphere,
 * generating a random state for the drone, then translating the drone's base in order to nullify
 * the distance between the end-effector position (obtained through forward kinematics) and the sample's
 * position in the sphere.
 */
class DroneStateConstraintSampler : public constraint_samplers::ConstraintSampler {

    std::string name_;
    std::string ee_name_;

    std::optional<SphericalConstraint> ee_target_;

public:
    DroneStateConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene,
                                const std::string &groupName);

    bool configure(const moveit_msgs::Constraints &constr) override;

    bool sample(moveit::core::RobotState &state,
                const moveit::core::RobotState &reference_state,
                unsigned int max_attempts) override;

    bool project(moveit::core::RobotState &state, unsigned int max_attempts) override;

    const std::string &getName() const override;

    static void randomizeUprightWithBase(moveit::core::RobotState &state, double translation_bound);

    static void moveEndEffectorToGoal(moveit::core::RobotState &state, double tolerance,
                                      const Eigen::Vector3d &target);
};

class DroneStateConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator {
    constraint_samplers::ConstraintSamplerPtr
    alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
          const moveit_msgs::Constraints &constr) override {

        auto sampler = std::make_shared<DroneStateConstraintSampler>(scene, group_name);

        sampler->configure(constr);

        return sampler;
    }

    [[nodiscard]] bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
                                  const moveit_msgs::Constraints &constr) const override {

        return constr.orientation_constraints.size() == 1 && constr.orientation_constraints[0].link_name == "base_link"
               && (constr.position_constraints.empty() || (constr.position_constraints.size() == 1 &&
                                                           constr.position_constraints[0].link_name == "end_effector"));
    }

};

#endif //NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H
