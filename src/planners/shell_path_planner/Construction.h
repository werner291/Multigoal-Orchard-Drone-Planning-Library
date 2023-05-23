/**
 * @file
 * @brief Header file containing convenience functions for creating OmplShellSpaces from AppleTreePlanningScenes.
 *
 * This header provides a collection of convenience functions that construct OmplShellSpaces using different strategies.
 * Each function takes an AppleTreePlanningScene and returns a corresponding OmplShellSpace. These functions conform to
 * the MkOmplShellFn template.
 */

#ifndef NEW_PLANNERS_CONSTRUCTION_H
#define NEW_PLANNERS_CONSTRUCTION_H

#include "../../AppleTreePlanningScene.h"
#include "../../shell_space/OmplShellSpace.h"
#include "../../utilities/experiment_utils.h"
#include "../../shell_space/SphereShell.h"
#include "../../shell_space/CuttingPlaneConvexHullShell.h"
#include "../../shell_space/CGALMeshShell.h"
#include "../../shell_space/CylinderShell.h"

/**
 * A method/strategy for constructing a WorkspaceShell from scene information.
 *
 * @tparam ShellPoint 		The type of points in the shell.
 */
template <typename ShellPoint>
using MkWorkspaceShellFn = std::function<std::shared_ptr<WorkspaceShell<ShellPoint>>>;



/**
 * @brief Constructs an OmplShellSpace with a cutting plane convex hull around the leaves of the apple tree.
 * This function conforms to the MkOmplShellFn template.
 *
 * @param scene_info The AppleTreePlanningScene containing information about the apple tree.
 * @param si The ompl::base::SpaceInformationPtr for the constructed OmplShellSpace.
 * @return A shared_ptr to the OmplShellSpace containing the cutting plane convex hull.
 */
std::shared_ptr<OmplShellSpace<ConvexHullPoint>>
cuttingPlaneChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si);

/**
 * @brief Constructs an OmplShellSpace with a CGAL-based convex hull around the leaves of the apple tree.
 * This function conforms to the MkOmplShellFn template.
 *
 * @param scene_info The AppleTreePlanningScene containing information about the apple tree.
 * @param si The ompl::base::SpaceInformationPtr for the constructed OmplShellSpace.
 * @return A shared_ptr to the OmplShellSpace containing the CGAL-based convex hull.
 */
std::shared_ptr<OmplShellSpace<CGALMeshShellPoint>>
cgalChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si);

/**
 * @brief Constructs an OmplShellSpace with a cylindrical shell around the leaves of the apple tree.
 * This function conforms to the MkOmplShellFn template.
 *
 * @param scene The AppleTreePlanningScene containing information about the apple tree.
 * @param si The ompl::base::SpaceInformationPtr for the constructed OmplShellSpace.
 * @return A shared_ptr to the OmplShellSpace containing the cylindrical shell.
 */
std::shared_ptr<OmplShellSpace<CylinderShellPoint>>
cylinderShell(const AppleTreePlanningScene &scene, const ompl::base::SpaceInformationPtr &si);

/**
 * @brief Constructs an OmplShellSpace with a cutting plane convex hull around the leaves of the apple tree.
 * This function conforms to the MkOmplShellFn template.
 *
 * @param scene_info The AppleTreePlanningScene containing information about the apple tree.
 * @param si The ompl::base::SpaceInformationPtr for the constructed OmplShellSpace.
 * @return A shared_ptr to the OmplShellSpace containing the cutting plane convex hull.
 */
std::shared_ptr<OmplShellSpace<Eigen::Vector3d>>
minimumEnclosingSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si);

#endif //NEW_PLANNERS_CONSTRUCTION_H
