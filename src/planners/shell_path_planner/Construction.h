
#ifndef NEW_PLANNERS_CONSTRUCTION_H
#define NEW_PLANNERS_CONSTRUCTION_H

#include "../../planning_scene_diff_message.h"
#include "../../shell_space/OmplShellSpace.h"
#include "../../utilities/experiment_utils.h"
#include "../../shell_space/SphereShell.h"

/**
 * A method/strategy for constructing a WorkspaceShell from scene information.
 *
 * @tparam ShellPoint 		The type of points in the shell.
 */
template <typename ShellPoint>
using MkWorkspaceShellFn = std::function<std::shared_ptr<WorkspaceShell<ShellPoint>>>;

/**
 * A method/strategy for constructing a OmplShellSpace from scene information.
 */
template <typename ShellPoint>
using MkOmplShellFn = std::function<std::shared_ptr<OmplShellSpace<ShellPoint>>(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr&)>;



#endif //NEW_PLANNERS_CONSTRUCTION_H
