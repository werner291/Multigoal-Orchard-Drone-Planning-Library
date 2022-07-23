#ifndef NEW_PLANNERS_COLLISIONFREESHELL_H
#define NEW_PLANNERS_COLLISIONFREESHELL_H

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathGeometric.h>

/**
 * A model of some retraction of the planning space that may serve as a "highway" to rapidly traverse the planning space.
 */
template<typename ShellPoint>
class CollisionFreeShell {

	ompl::base::SpaceInformationPtr si;

public:
	/// Given a shell point, return the corresponding state.
	[[nodiscard]] virtual ompl::base::ScopedState<> state_on_shell(const ShellPoint &a) const = 0;

	/// Given a goal, return the corresponding shell point.
	[[nodiscard]] virtual ShellPoint shell_point(const ompl::base::Goal *a) const = 0;

	/// Given a state, return the corresponding shell point.
	[[nodiscard]] virtual ShellPoint shell_point(const ompl::base::ScopedState<> &a) const = 0;

	/// Given two shell points, return a path composed of a sequence of states.
    [[nodiscard]] virtual ompl::geometric::PathGeometric path_on_shell(const ShellPoint& a, const ShellPoint& b) const = 0;

	/// Given two shell points, return the distance between them.
	[[nodiscard]] virtual double predict_path_length(const ShellPoint& a, const ShellPoint& b) const = 0;

};

#endif //NEW_PLANNERS_COLLISIONFREESHELL_H
