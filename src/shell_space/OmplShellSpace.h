
#ifndef NEW_PLANNERS_OMPLSHELLSPACE_H
#define NEW_PLANNERS_OMPLSHELLSPACE_H

#include <memory>
#include <utility>
#include "MoveItShellSpace.h"
#include "../ompl_custom.h"

/**
 * @brief A class that represents a shell space for OMPL (Open Motion Planning Library).
 *
 * A shell space is a simplified representation of a workspace that can be used for motion planning.
 * This class provides methods to convert points from the shell space to OMPL states and vice versa,
 * compute paths in the shell space, predict path lengths, and find the shell point closest to a given state or goal.
 *
 * Class is a decorator for MoveItShellSpace.
 *
 * @tparam ShellPoint The type of point used in the shell space.
 */
template<typename ShellPoint>
class OmplShellSpace {

	/// All hail the SpaceInformation God class, may thy references
	/// be omnipresent thoughout the codebase.
	ompl::base::SpaceInformationPtr si;

	/// The underlying MoveItShellSpace object that this class is a wrapper for.
	std::shared_ptr<const MoveItShellSpace<ShellPoint>> shell_space;

public:
	/**
	 * @brief Constructs an OmplShellSpace object.
	 *
	 * @param si The OMPL space information object that will be used for motion planning.
	 * @param shellSpace A shared pointer to the MoveItShellSpace object that defines the shell space.
	 */
	OmplShellSpace(ompl::base::SpaceInformationPtr si,
				   const std::shared_ptr<const MoveItShellSpace<ShellPoint>> &shellSpace);

	/**
	 * @brief Converts a point from the shell space to an OMPL state.
	 *
	 * This method takes a point in the shell space and converts it to an OMPL state that can be used for motion planning.
	 *
	 * @param point The point to convert.
	 * @param output A pointer to the output OMPL state.
	 */
	void stateFromPoint(const ShellPoint &point, ompl::base::State *output) const;

	/**
	 * @brief Computes a geometric path in the shell space.
	 *
	 * This method computes a geometric path in the shell space between two given points.
	 *
	 * @param start The starting point of the path.
	 * @param end The end point of the path.
	 * @return A PathGeometric object that represents the computed path.
	 */
	ompl::geometric::PathGeometric shellPath(const ShellPoint &start, const ShellPoint &end) const;

	/**
	 * @brief Predicts the length of a path in the shell space.
	 *
	 * This method predicts the length of a path in the shell space between two given points.
	 *
	 * @param start The starting point of the path.
	 * @param end The end point of the path.
	 * @return The predicted length of the path.
	 */
	double predict_path_length(const ShellPoint &start, const ShellPoint &end) const;

	/**
	 * @brief Constructs an OmplShellSpace object from a WorkspaceShell object.
	 *
	 * This static method constructs an OmplShellSpace object from a WorkspaceShell object.
	 *
	 * @param shell A shared pointer to the WorkspaceShell object.
	 * @param si The OMPL space information object that will be used for motion planning.
	 * @return A shared pointer to the constructed OmplShellSpace object.
	 */
	static std::shared_ptr<OmplShellSpace>
	fromWorkspaceShell(const std::shared_ptr<const WorkspaceShell<ShellPoint>> &shell,
					   const ompl::base::SpaceInformationPtr &si);

	/**
	 * @brief Finds the shell point closest to a given OMPL state.
	 *
	 * This method finds the shell point in the shell space that is closest to a given OMPL state.
	 *
	 * @param state A pointer to the OMPL state.
	 * @return The shell point closest to the OMPL state.
	 */
	ShellPoint pointNearState(const ompl::base::State *state) const;

	/**
	 * @brief Finds the shell point closest to a given OMPL goal.
	 *
	 * This method finds the shell point in the shell space that is closest to a given OMPL goal.
	 *
	 * @param goal A pointer to the OMPL goal.
	 * @return The shell point closest to the OMPL goal.
	 */
	ShellPoint pointNearGoal(const ompl::base::Goal *goal) const;

};



#endif //NEW_PLANNERS_OMPLSHELLSPACE_H
