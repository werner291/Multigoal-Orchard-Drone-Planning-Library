
#ifndef NEW_PLANNERS_GOAL_TO_GOAL_VIA_SHELL_H
#define NEW_PLANNERS_GOAL_TO_GOAL_VIA_SHELL_H

#include "StateVectorPath.h"
#include "approach_paths.h"
#include "path_traits.h"
#include <optional>
#include <functional>
#include <vector>
#include <type_traits>

/**
 *
 * @tparam ShellSpace	The type of the shell space.
 * @tparam Path	        The type of the path.
 *
 * @param a1			The first approach path.
 * @param shell_space	The shell space.
 * @param a2		    The second approach path.
 * @return				The concatenation of the two approach paths.
 */
template<
		typename ShellSpace,
		typename Path
>
Path assemble_goal_to_goal_path(
		const ApproachPath<Path, ShellSpace>& a1,
		const ShellSpace& shell_space,
		const ApproachPath<Path, ShellSpace>& a2
		) {

	const auto shell_path = shell_state_traits<ShellSpace>::path(a1.shell_point, a2.shell_point);

	return path_traits<Path>::concatenate_path({
		path_traits<Path>::reverse(a1.path),
		shell_path,
		a2.path
	});

}

#endif //NEW_PLANNERS_GOAL_TO_GOAL_VIA_SHELL_H
