#ifndef NEW_PLANNERS_SPHERESHELL_H
#define NEW_PLANNERS_SPHERESHELL_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/Goal.h>
#include "procedural_tree_generation.h"
#include "moveit_conversions.h"
#include "ompl_custom.h"

/**
 * Describes, in MoveIt terms, a "shell" shape that fits around the obstacles in the scene,
 * and can be used to quickly plan collision-free paths around then.
 */
class CollisionFreeShell {

public:
	/**
	 * Construct a RobotState located on the collision-free shell,
	 * in a point corresponding to the given shell point.
	 *
	 * @param drone 	A robot model.
	 * @param a 		The shell point (assumed to be on the shell)
	 * @return 			A RobotState located on the collision-free shell.
	 */
    [[nodiscard]] virtual moveit::core::RobotState state_on_shell(
			const moveit::core::RobotModelConstPtr &drone,
			const Eigen::Vector3d &a) const = 0;

	/**
	 * Construct a path from a to b, on the collision-free shell.
	 *
	 * @param drone 	A robot model (assumed to be on the shell)
	 * @param a 		The start point (assumed to be on the shell).
	 * @param b 		The end point (assumed to be on the shell).
	 * @return 			The path.
	 */
    [[nodiscard]] virtual std::vector<moveit::core::RobotState> path_on_shell(
			const moveit::core::RobotModelConstPtr &drone,
			const Eigen::Vector3d &a,
			const Eigen::Vector3d &b) const = 0;

	/**
	 * Project a point from anywhere in R^3 onto the collision-free shell.
	 *
	 * @param a 		The point to project.
	 * @return 			The point on the shell.
	 */
    [[nodiscard]] virtual Eigen::Vector3d project(
			const Eigen::Vector3d &a) const = 0;

	/**
	 * Predict the length of a path from a to b, on the collision-free shell, without constructing the path.
	 *
	 * This prediction may be an approximation.
	 *
	 * @param a 		The start point (assumed to be on the shell).
	 * @param b 		The end point (assumed to be on the shell).
	 * @return 			The predicted length.
	 */
	[[nodiscard]] virtual double predict_path_length(
			const Eigen::Vector3d &a,
			const Eigen::Vector3d &b) const = 0;

};

/**
 * An implementation of a CollisionFreeShell, using a sphere as the shell shape.
 */
class SphereShell : public CollisionFreeShell {

	/// The center of the sphere.
    Eigen::Vector3d center;
	/// The radius of the sphere.
    double radius;

public:
	/**
	 * Construct a SphereShell.
	 * @param center 	The center of the sphere.
	 * @param radius 	The radius of the sphere.
	 */
    SphereShell(Eigen::Vector3d center, double radius);

	/**
	 * Construct a RobotState located on the collision-free shell,
	 * in a point corresponding to the given shell point.
	 *
	 * In this case, that is: a state where the end-effector of the robot is at the given point,
	 * the robot is facing the center of the sphere, with the arm extended and horizontal.
	 *
	 * @param drone 	A robot model.
	 * @param a 		The shell point (assumed to be on the shell)
	 * @return 			A RobotState located on the collision-free shell.
	 */
    [[nodiscard]] moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const override;

	/**
	 * Construct a path from a to b from shell states along the geodesic between a and b.
	 *
	 * @param drone 	A robot model (assumed to be on the shell)
	 * @param a 		The start point (assumed to be on the shell).
	 * @param b 		The end point (assumed to be on the shell).
	 * @return 			The path.
	 */
    [[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;

	/**
	 * Perform a central projection of a point from anywhere in R^3 onto the sphere surface.
	 *
	 * @param a 		The point to project.
	 * @return 			The point on the sphere,
	 */
    [[nodiscard]] Eigen::Vector3d project(const Eigen::Vector3d &a) const override;

	/**
	 * Predict the length of a path from a to b; this is the length of the geodesic between a and b.
	 *
	 * @param a 		The start point (assumed to be on the shell).
	 * @param b 		The end point (assumed to be on the shell).
	 * @return 			The predicted length.
	 */
	[[nodiscard]] double predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;
};

/**
 * An adapter that translates between OMPL concepts and CollisionFreeShell (which uses MoveIt types).
 *
 * Assumes that the OMPL state space is a ModelBasedStateSpace.
 */
class OMPLSphereShellWrapper {

	/// Reference to the underlying CollisionFreeShell.
    std::shared_ptr<CollisionFreeShell> shell;

	/// The OMPL SpaceInformation.
    ompl::base::SpaceInformationPtr si;

public:
    [[nodiscard]] std::shared_ptr<CollisionFreeShell> getShell() const;

public:
    OMPLSphereShellWrapper(std::shared_ptr<CollisionFreeShell> shell, ompl::base::SpaceInformationPtr si);

	void state_on_shell(const Eigen::Vector3d& a, ompl::base::State* st) const;

    void state_on_shell(const ompl::base::Goal* apple, ompl::base::State* st) const;

	ompl::geometric::PathGeometric path_on_shell(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

	ompl::geometric::PathGeometric path_on_shell(const ompl::base::Goal* a, const ompl::base::Goal* b);

	[[nodiscard]] double predict_path_length(const ompl::base::Goal* a, const ompl::base::Goal* b) const;

	[[nodiscard]] double predict_path_length(const ompl::base::State* a, const ompl::base::Goal* b) const;


};

#endif //NEW_PLANNERS_SPHERESHELL_H
