//
// Created by werner on 13-08-21.
//

#ifndef NEW_PLANNERS_ENDEFFECTORCONSTRAINTSAMPLER_H
#define NEW_PLANNERS_ENDEFFECTORCONSTRAINTSAMPLER_H

#include <random_numbers/random_numbers.h>

class EndEffectorPositionConstraintSampler : public constraint_samplers::ConstraintSampler {

    std::string name_;
    std::string ee_name_;

    Eigen::Vector3d ee_target_{0.0, 0.0, 0.0};
    double radius_ = 0.0;

public:
    EndEffectorPositionConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene,
                                         const std::string &groupName);

    bool configure(const moveit_msgs::Constraints &constr) override;

    bool sample(moveit::core::RobotState &state,
                const moveit::core::RobotState &reference_state,
                unsigned int max_attempts) override;

    bool project(moveit::core::RobotState &state, unsigned int max_attempts) override;

    void move_to_within(moveit::core::RobotState &state, double radius) const;

    const std::string &getName() const override;

};

class EndEffectorPositionConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator {
    constraint_samplers::ConstraintSamplerPtr
    alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
          const moveit_msgs::Constraints &constr) override {

        auto sampler = std::make_shared<EndEffectorPositionConstraintSampler>(scene, group_name);

        sampler->configure(constr);

        return sampler;
    }

    [[nodiscard]] bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
                                  const moveit_msgs::Constraints &constr) const override {
        return constr.position_constraints.size() == 1 && constr.position_constraints[0].link_name == "end_effector";
    }

};

#endif //NEW_PLANNERS_ENDEFFECTORCONSTRAINTSAMPLER_H
