
#ifndef NEW_PLANNERS_MOVEITAPPLESPHERESHELL_H
#define NEW_PLANNERS_MOVEITAPPLESPHERESHELL_H

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include <Eigen/Core>
#include "CollisionFreeShell.h"

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/ScopedState.h>

class MoveItAppleSphereShell : public CollisionFreeShell<Eigen::Vector3d> {

	bodies::BoundingSphere sphere;

public:
	[[nodiscard]] ompl::base::ScopedState<> state_on_shell(const Eigen::Vector3d &a) const override;

	[[nodiscard]] Eigen::Vector3d shell_point(const ompl::base::Goal *const &a) const override;

	[[nodiscard]] Eigen::Vector3d shell_point(const ompl::base::ScopedState<> &a) const override;

	[[nodiscard]] ompl::base::PAth path_on_shell(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;

	[[nodiscard]] double predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;

	[[nodiscard]] Eigen::Vector3d project(const Eigen::Vector3d &a) const;

};

#endif //NEW_PLANNERS_MOVEITAPPLESPHERESHELL_H
