#ifndef NEW_PLANNERS_SPHERESHELL_H
#define NEW_PLANNERS_SPHERESHELL_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/Goal.h>
#include "../procedural_tree_generation.h"
#include "../utilities/moveit_conversions.h"
#include "../ompl_custom.h"
#include "WorkspaceShell.h"

class SphereSurfaceArmVector {
public:
	Eigen::Vector3d arm_vector(const Eigen::Vector3d &p) const override;

	Eigen::Vector3d nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	Eigen::Vector3d surface_point(const Eigen::Vector3d &p) const override;
};
//
///**
// * An implementation of a ShellSpace, using a sphere as the shell shape.
// */
//class SphereShell : public CollisionFreeShell<Eigen::Vector3d> {
//
//	/// The center of the sphere.
//    Eigen::Vector3d center;
//	/// The radius of the sphere.
//    double radius;
//
//	/// Whether to place the arm horizontal or not.
//	bool horizontal_arm;
//
//	/**
//	 * Perform a central projection of a point from anywhere in R^3 onto the sphere surface.
//	 *
//	 * @param a 		The point to project.
//	 * @return 			The point on the sphere,
//	 */
//	[[nodiscard]] Eigen::Vector3d project(const Eigen::Vector3d &a) const;
//
//public:
//	/**
//	 * Construct a SphereShell.
//	 * @param center 	The center of the sphere.
//	 * @param radius 	The radius of the sphere.
//	 */
//	SphereShell(Eigen::Vector3d center, double radius, bool horizontalArm);
//
//	/**
//	 * Construct a RobotState located on the collision-free shell,
//	 * in a point corresponding to the given shell point.
//	 *
//	 * In this case, that is: a state where the end-effector of the robot is at the given point,
//	 * the robot is facing the center of the sphere, with the arm extended and horizontal.
//	 *
//	 * @param drone 	A robot model.
//	 * @param a 		The shell point (assumed to be on the shell)
//	 * @return 			A RobotState located on the collision-free shell.
//	 */
//    [[nodiscard]] moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const override;
//
//	/**
//	 * Construct a path from a to b from shell states along the geodesic between a and b.
//	 *
//	 * @param drone 	A robot model (assumed to be on the shell)
//	 * @param a 		The start point (assumed to be on the shell).
//	 * @param b 		The end point (assumed to be on the shell).
//	 * @return 			The path.
//	 */
//    [[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;
//
//	/**
//	 * Predict the length of a path from a to b; this is the length of the geodesic between a and b.
//	 *
//	 * @param a 		The start point (assumed to be on the shell).
//	 * @param b 		The end point (assumed to be on the shell).
//	 * @return 			The predicted length.
//	 */
//	[[nodiscard]] double predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;
//
//	Eigen::Vector3d gaussian_sample_near_point(const Eigen::Vector3d &near) const override;
//
//	Eigen::Vector3d project(const moveit::core::RobotState &st) const override;
//
//	Eigen::Vector3d project(const Apple &st) const override;
//};
//
///**
// * An adapter that translates between OMPL concepts and ShellSpace (which uses MoveIt types).
// *
// * Assumes that the OMPL state space is a ModelBasedStateSpace.
// */
//template<typename ShellPoint>
//class OMPLShellSpaceWrapper {
//
//	/// Reference to the underlying ShellSpace.
//    std::shared_ptr<CollisionFreeShell<ShellPoint>> shell;
//
//	/// The OMPL SpaceInformation.
//    ompl::base::SpaceInformationPtr si;
//
//public:
//    OMPLShellSpaceWrapper(std::shared_ptr<CollisionFreeShell < ShellPoint> > shell,
//						   ompl::base::SpaceInformationPtr si) : shell(std::move(shell)), si(std::move(si)) {
//	}
//
//	void state_on_shell(const ShellPoint& a, ompl::base::State* st) const {
//
//		auto state_space = std::dynamic_pointer_cast<ompl_interface::ModelBasedStateSpace>(si->getStateSpace());
//
//		state_space->copyToOMPLState(st, shell->state_on_shell(state_space->getRobotModel(), a));
//	}
//
//
//	ompl::geometric::PathGeometric path_on_shell(const ShellPoint& a, const ShellPoint& b)  {
//		return omplPathFromMoveitTrajectory(shell->path_on_shell(si->getStateSpace()
//																		 ->as<ompl_interface::ModelBasedStateSpace>()
//																		 ->getRobotModel(), a, b), si);
//	}
//
//	std::shared_ptr<CollisionFreeShell<ShellPoint>> getShell() const {
//		return shell;
//	}
//
//	void state_on_shell(const ompl::base::Goal *apple, ompl::base::State *st) const {
//		state_on_shell(project(apple), st);
//	}
//
//	ompl::geometric::PathGeometric path_on_shell(const ompl::base::Goal *a, const ompl::base::Goal *b) {
//		return path_on_shell(project(a), project(b));
//	}
//
//	double predict_path_length(const ompl::base::Goal *a, const ompl::base::Goal *b) const {
//		return shell->predict_path_length(project(a), project(b));
//	}
//
//	double predict_path_length(const ompl::base::State *a, const ompl::base::Goal *b) const {
//		return shell->predict_path_length(project(a), project(b));
//	}
//
//	ShellPoint gaussian_sample_near_point(const ShellPoint &near) const {
//		return shell->gaussian_sample_near_point(near);
//	}
//
//	ShellPoint project(const ompl::base::State *st) const {
//		moveit::core::RobotState rst(si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->getRobotModel());
//		si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(rst, st);
//		rst.update(true);
//
//
//		if (isnan(rst.getGlobalLinkTransform("end_effector").translation().z())) {
//			std::cout << rst << std::endl;
//		}
//
//		return shell->project(rst);
//	}
//
//	ShellPoint project(const ompl::base::Goal *goal) const {
//		return shell->project(Apple {
//				goal->as<DroneEndEffectorNearTarget>()->getTarget(),
//		});
//	}
//
//};
//
//moveit::core::RobotState robotStateFromFacing(const moveit::core::RobotModelConstPtr &drone,
//											  const Eigen::Vector3d &desired_ee_pos,
//											  const Eigen::Vector3d &required_facing);



#endif //NEW_PLANNERS_SPHERESHELL_H
