
#include <utility>

#include <boost/asio.hpp>

#include <range/v3/all.hpp>
#include <range/v3/view/drop.hpp>

#include <vtkProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkRendererCollection.h>

#include "../utilities/vtk.h"
#include "../utilities/mesh_utils.h"
#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/SphereShell.h"
#include "../shell_space/CGALMeshShell.h"
#include "../utilities/enclosing_sphere.h"

#include "../visualization/path_visualization.h"
#include "../visualization/LabeledActors.h"
#include "../visualization/compute_prm.h"
#include "../visualization/SimpleVtkViewer.h"

#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"


int main(int argc, char **argv) {

	SimpleVtkViewer viewer;

	auto current_tree_models = loadTreeMeshes("appletree");

    viewer.addActor(createColoredMeshActor(current_tree_models.trunk_mesh, {0.5, 0.3, 0.1, 1.0}, true));
    viewer.addActor(createColoredMeshActor(current_tree_models.leaves_mesh, {0.1, 0.5, 0.1, 1.0}, true));

    for (const auto &mesh : current_tree_models.fruit_meshes) {
        viewer.addActor(createColoredMeshActor(mesh, {0.9, 0.0, 0.0, 1.0}, true));
    }

    std::vector<Apple> apples;

    for (const auto &mesh : current_tree_models.fruit_meshes)
    {
        apples.push_back(appleFromMesh(mesh));
    }

    AppleTreePlanningScene scene {
        .scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(treeMeshesToMoveitSceneMsg(current_tree_models, false))),
        .apples = apples
    };
	// auto sphere_shell = paddedSphericalShellAroundLeaves(scene, 0.0);
    // auto sphere_actor = mkSphereShellActor(*sphere_shell);
    // viewer.addActor(sphere_actor);

	auto convex_hull = convexHull(utilities::extract_leaf_vertices(scene));
	auto cutting_plane_chull_shell = std::make_shared<CuttingPlaneConvexHullShell>(convex_hull, 0.0, 0.0);
	// auto cgal_mesh_shell = std::make_shared<CGALMeshShell>(convex_hull, 0.0, 0.0);
	auto chull_actor = createColoredMeshActor(convex_hull, {0.8, 0.8, 0.8, 0.2}, true);
    viewer.addActor(chull_actor);

	VtkLineSegmentsVisualization path_viz(1.0, 0.0, 1.0);
	viewer.addActor(path_viz.getActor());

	size_t from_apple = 0;
    size_t to_apple = 42;

    std::vector<Eigen::Vector3d> path = idealizedPathViaShell(*cutting_plane_chull_shell, apples[from_apple].center, apples[to_apple].center, 32);

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> path_segments;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        path_segments.emplace_back(path[i], path[i + 1]);
    }

    path_viz.updateLine(path_segments);

    auto robot = loadRobotModel();
    auto start_state = randomStateOutsideTree(robot, 42);

	auto ss = omplStateSpaceForDrone(robot);
	auto si = loadSpaceInformation(ss, scene);

    {

        MkOmplShellFn<ConvexHullPoint> planner_allocator = [&](const AppleTreePlanningScene& scene, const ompl::base::SpaceInformationPtr& si) -> std::shared_ptr<OmplShellSpace<ConvexHullPoint>> {
            auto workspaceShell = horizontalAdapter<ConvexHullPoint>(cutting_plane_chull_shell);
	        return OmplShellSpace<ConvexHullPoint>::fromWorkspaceShell(workspaceShell, si);
        };

        ShellPathPlanner<ConvexHullPoint> planner(
            planner_allocator, 
            std::make_unique<MakeshiftPrmApproachPlanningMethods<ConvexHullPoint>>(si), 
            true, 
            DistancePredictionStrategy::SHELL_PATH_LENGTH);

        ompl::base::ScopedState<> start(ss);
        ss->copyToOMPLState(start.get(), start_state);

        std::vector<ompl::base::GoalPtr> goals;

        for (const auto &apple : apples) {
            goals.push_back(std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center));
        }

        auto ptc = ompl::base::plannerNonTerminatingCondition();
        auto path = planner.plan(si, start.get(), goals, scene, ptc);

        std::vector<Eigen::Vector3d> ee_trace;
		std::vector<Eigen::Vector3d> base_trace;

        for (const auto &segment: path.segments) {
            for (size_t state_i = 0; state_i < segment.path_.getStateCount(); ++state_i) {

                auto state = segment.path_.getState(state_i);
                
                moveit::core::RobotState robot_state(robot);

                ss->copyToRobotState(robot_state, state);

                ee_trace.push_back(robot_state.getGlobalLinkTransform("end_effector").translation());
				base_trace.push_back(robot_state.getGlobalLinkTransform("base_link").translation());

            }
        }

        VtkPolyLineVisualization ee_trace_viz(1.0, 0.0, 0.0);
        ee_trace_viz.updateLine(ee_trace);
		viewer.addActor(ee_trace_viz.getActor());

		VtkPolyLineVisualization base_trace_viz(0.0, 1.0, 0.0);
		base_trace_viz.updateLine(base_trace);
		viewer.addActor(base_trace_viz.getActor());

		VtkLineSegmentsVisualization path_viz(1.0, 1.0, 0.0);

		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> path_segments;
		for (size_t i = 0; i < ee_trace.size(); ++i) {
			path_segments.emplace_back(ee_trace[i], base_trace[i]);
		}
		path_viz.updateLine(path_segments);
		viewer.addActor(path_viz.getActor());

    }

    viewer.start();

}

