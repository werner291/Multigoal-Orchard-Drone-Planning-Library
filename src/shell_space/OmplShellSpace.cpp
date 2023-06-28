#include "OmplShellSpace.h"
#include "CuttingPlaneConvexHullShell.h"
#include "CGALMeshShell.h"
#include "CylinderShell.h"

template<typename ShellPoint>
ShellPoint OmplShellSpace<ShellPoint>::pointNearGoal(const ompl::base::Goal *goal) const {

	// Get the target position from the OMPL goal and create an Apple with zero velocity
	return shell_space->pointNearGoal(Apple{goal->as<DroneEndEffectorNearTarget>()->getTarget(), {0, 0, 0}});
}

template<typename ShellPoint>
ShellPoint OmplShellSpace<ShellPoint>::pointNearState(const ompl::base::State *state) const {

	// Create a RobotState from the OMPL state
	moveit::core::RobotState st(si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->getRobotModel());
	si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(st, state);

	// Get the shell point closest to the RobotState
	return shell_space->pointNearState(st);

}


template<typename ShellPoint>
std::shared_ptr<OmplShellSpace<ShellPoint>>
OmplShellSpace<ShellPoint>::fromWorkspaceShell(const std::shared_ptr<const WorkspaceShell<ShellPoint>> &shell,
											   const ompl::base::SpaceInformationPtr &si) {

	// Create a MoveIt shell space
	auto moveit_shell_space = std::make_shared<MoveItShellSpace<ShellPoint>>(si->getStateSpace()
																					 ->as<ompl_interface::ModelBasedStateSpace>()
																					 ->getRobotModel(), shell);

	// Create an OMPL shell space
	return std::make_shared<OmplShellSpace<ShellPoint>>(si, moveit_shell_space);

}

template<typename ShellPoint>
double OmplShellSpace<ShellPoint>::predict_path_length(const ShellPoint &start, const ShellPoint &end) const {
	return shell_space->predict_path_length(start, end);
}

template<typename ShellPoint>
std::vector<std::vector<double>>
OmplShellSpace<ShellPoint>::predict_path_lengths(const std::vector<ShellPoint> &points) const {
	return shell_space->distance_matrix(points);
}

template<typename ShellPoint>
ompl::geometric::PathGeometric
OmplShellSpace<ShellPoint>::shellPath(const ShellPoint &start, const ShellPoint &end) const {

	// Get the MoveIt path from the shell space
	auto path = shell_space->shellPath(start, end);

	// Convert the MoveIt path to an OMPL path
	return robotPathToOmplPath(path, si);

}

template<typename ShellPoint>
void OmplShellSpace<ShellPoint>::stateFromPoint(const ShellPoint &point, ompl::base::State *output) const {

	// Get the MoveIt state from the shell space
	auto st = shell_space->stateFromPoint(point);

	// Copy the drone state from the shell state to the output state.
	si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToOMPLState(output, st);

}

template<typename ShellPoint>
OmplShellSpace<ShellPoint>::OmplShellSpace(ompl::base::SpaceInformationPtr si,
										   const std::shared_ptr<const MoveItShellSpace<ShellPoint>> &shellSpace)
		: si(std::move(si)), shell_space(shellSpace) {
}

template
class OmplShellSpace<Eigen::Vector3d>;

template
class OmplShellSpace<ConvexHullPoint>;

template
class OmplShellSpace<mgodpl::cgal_utils::CGALMeshPointAndNormal>;

template
class OmplShellSpace<CylinderShellPoint>;