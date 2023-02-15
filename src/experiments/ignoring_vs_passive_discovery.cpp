#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../planners/ShellPathPlanner.h"
#include "../vtk/Viewer.h"
#include "../CurrentPathState.h"

#include "../vtk/SimpleVtkViewer.h"
#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/goal_events.h"
#include "../DynamicGoalVisitationEvaluation.h"

#include <vtkProperty.h>

struct PlanningProblem {
	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
};

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
}

void createActors(const TreeMeshes &meshes,
				  const std::vector<AppleDiscoverabilityType> &apple_discoverability,
				  std::vector<vtkActor *> &apple_actors,
				  SimpleVtkViewer &viewer) {
	vtkNew<vtkActorCollection> actors;

	auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
	setColorsByEncoding(tree_actor, TRUNK_RGB, false);

	auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
	setColorsByEncoding(leaves_actor, LEAVES_RGB, false);

	actors->AddItem(tree_actor);
	actors->AddItem(leaves_actor);

	for (const auto &[fruit_index, fruit_mesh]: meshes.fruit_meshes | ranges::views::enumerate) {
		auto fruit_actor = createActorFromMesh(fruit_mesh);

		std::array<double, 3> rgb{0.0, 0.0, 0.0};

		if (apple_discoverability[fruit_index]) {
			rgb = {1.0, 0.0, 0.0};
		} else {
			rgb = {0.0, 0.0, 1.0};
		}

		fruit_actor->GetProperty()->SetDiffuseColor(rgb.data());

		actors->AddItem(fruit_actor);

		apple_actors.push_back(fruit_actor);
	}

	viewer.addActorCollection(actors);
}

int main(int argc, char **argv) {

	TreeMeshes meshes = loadTreeMeshes("appletree");

	meshes.fruit_meshes.resize(10);

	const auto scene_msg = treeMeshesToMoveitSceneMsg(meshes);

	const auto scene = AppleTreePlanningScene{scene_msg, meshes.fruit_meshes | ranges::views::transform(appleFromMesh) |
														 ranges::to_vector};

	const auto robot = loadRobotModel();
	const int N_INITIAL_STATES = 100;

	using namespace ranges;

	boost::asio::thread_pool pool(4);

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = std::make_shared<MoveItShellSpace<Eigen::Vector3d >>(robot, workspaceShell);

	const auto start_state = randomStateOutsideTree(robot, 0);
	auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size(), 1.0);

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, scene);

	auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

	auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

		auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

		return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	};

	auto static_planner = std::make_shared<ShellPathPlanner<Eigen::Vector3d>>(shellBuilder,
																			  std::move(approach_methods),
																			  true);

	auto change_ignoring = std::make_shared<ChangeIgnoringReplannerAdapter>(static_planner);

	DynamicGoalVisitationEvaluation eval(change_ignoring, start_state, scene, apple_discoverability, si);

	robot_trajectory::RobotTrajectory traj = *eval.computeNextTrajectory();

	std::vector<vtkActor *> apple_actors;

	// Create a VtkRobotmodel object to visualize the robot itself.
	VtkRobotmodel robotModel(robot, start_state);

	SimpleVtkViewer viewer;

	// Add the robot model to the viewer.
	viewer.addActorCollection(robotModel.getLinkActors());

	// Add the tree meshes to the viewer.
	createActors(meshes, apple_discoverability, apple_actors, viewer);

	double time = 0.0;

	// The "main loop" of the program, called every frame.
	auto callback = [&]() {

		time += 0.01;

		if (eval.getRe() && eval.getRe()->at_t > time) {
			time = 0.0;
			traj = *eval.computeNextTrajectory();
		}

		// Update the robot's visualization to match the current state.
		moveit::core::RobotState state(robot);

		setStateToTrajectoryPoint(state, time, traj);

		robotModel.applyState(state);

		for (const auto &[apple_actor, apple]: views::zip(apple_actors, scene.apples)) {
			if ((state.getGlobalLinkTransform("end_effector").translation() - apple.center).norm() < 0.05) {
				apple_actor->GetProperty()->SetDiffuseColor(0.0, 1.0, 0.0);
			}
		}

	};

	viewer.addTimerCallback(callback);

	viewer.start();

	return EXIT_SUCCESS;

}

