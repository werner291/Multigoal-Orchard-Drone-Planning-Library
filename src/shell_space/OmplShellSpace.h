
#ifndef NEW_PLANNERS_OMPLSHELLSPACE_H
#define NEW_PLANNERS_OMPLSHELLSPACE_H

#include <memory>
#include <utility>
#include "MoveItShellSpace.h"
#include "../ompl_custom.h"

template<typename ShellPoint>
class OmplShellSpace {

	ompl::base::SpaceInformationPtr si;
	std::shared_ptr<const MoveItShellSpace<ShellPoint>> shell_space;

public:
	OmplShellSpace(ompl::base::SpaceInformationPtr si, const std::shared_ptr<const MoveItShellSpace<ShellPoint>> &shellSpace)
			: si(std::move(si)), shell_space(shellSpace) {
	}

	void stateFromPoint(const ShellPoint &point, ompl::base::State* output) const {

		// Get the MoveIt state from the shell space
		auto st = shell_space->stateFromPoint(point);

		// Copy the drone state from the shell state to the output state.
		si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToOMPLState(output, st);

	}

	ompl::geometric::PathGeometric shellPath(const ShellPoint &start, const ShellPoint &end) const {

		// Get the MoveIt path from the shell space
		auto path = shell_space->shellPath(start, end);

		// Convert the MoveIt path to an OMPL path
		return robotPathToOmplPath(path, si);

	}

	double predict_path_length(const ShellPoint &start, const ShellPoint &end) const {
		return shell_space->predict_path_length(start, end);
	}

	static std::shared_ptr<OmplShellSpace> fromWorkspaceShell(const std::shared_ptr<const WorkspaceShell<ShellPoint>> &shell, const ompl::base::SpaceInformationPtr& si) {

		// Create a MoveIt shell space
		auto moveit_shell_space = std::make_shared<MoveItShellSpace<ShellPoint>>(
				si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->getRobotModel(),
				shell);

		// Create an OMPL shell space
		return std::make_shared<OmplShellSpace<ShellPoint>>(si, moveit_shell_space);

	}

	const ompl::base::SpaceInformationPtr &getSpaceInformation() const {
		return si;
	}

	ShellPoint pointNearState(const ompl::base::State *state) const {

		moveit::core::RobotState st(si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->getRobotModel());
		si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(st, state);

		return shell_space->pointNearState(st);

	}

	ShellPoint pointNearGoal(const ompl::base::Goal *goal) const {
		return shell_space->pointNearGoal(Apple {
			goal->as<DroneEndEffectorNearTarget>()->getTarget(),
			{0,0,0}
		});
	}
};


#endif //NEW_PLANNERS_OMPLSHELLSPACE_H
