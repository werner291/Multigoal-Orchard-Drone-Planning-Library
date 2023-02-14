#include <range/v3/all.hpp>
#include <boost/asio.hpp>

#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../planners/ShellPathPlanner.h"
#include "../vtk/VtkRobotModel.h"
#include "../vtk/Viewer.h"
#include "../CurrentPathState.h"

struct PlanningProblem {
	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
};

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
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
	auto shell = std::make_shared < MoveItShellSpace < Eigen::Vector3d >> (robot, workspaceShell);

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto threadLocalStateSpace = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this every time.
	auto si = loadSpaceInformation(threadLocalStateSpace, scene);

	const auto start_state = randomStateOutsideTree(robot, 0);
	auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size());

	auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

	auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

		auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

		return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	};

	auto static_planner = std::make_shared<ShellPathPlanner<Eigen::Vector3d>>(shellBuilder,
																			  std::move(approach_methods),
																			  true);

	ompl::base::ScopedState start(si);
	threadLocalStateSpace->copyToOMPLState(start.get(), start_state);

	std::vector<ompl::base::GoalPtr> goals =
			views::ints(0, (int) scene.apples.size()) | views::filter([&](int goal_id) {
				return apple_discoverability[goal_id];
			}) | views::transform([&](int goal_id) {
				auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, scene.apples[goal_id].center);
				return std::static_pointer_cast<ompl::base::Goal>(goal);
			}) | to_vector;

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	auto result = static_planner->plan(si, start.get(), goals, scene, ptc);

	RobotPath path = omplPathToRobotPath(result.combined());

	robot_trajectory::RobotTrajectory traj = robotPathToConstantSpeedRobotTrajectory(path, 1.0);

	// Create a VTK actor to visualize the robot itself
	VtkRobotmodel robotModel(robot, start_state);

	vtkNew <vtkRenderer> viewerRenderer;
	vtkNew <vtkRenderWindow> visualizerWindow;
	vtkNew <vtkRenderWindowInteractor> renderWindowInteractor;

	visualizerWindow->SetSize(800, 600);
	visualizerWindow->SetWindowName("PointCloud");
	visualizerWindow->AddRenderer(viewerRenderer);

	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);

	auto tree_actors = buildTreeActors(meshes, false);
	for (int i = 0; i < tree_actors->GetNumberOfItems(); i++) {
		viewerRenderer->AddActor(vtkActor::SafeDownCast(tree_actors->GetItemAsObject(i)));
	}

	for (int i = 0; i < robotModel.getLinkActors()->GetNumberOfItems(); i++) {
		viewerRenderer->AddActor(vtkActor::SafeDownCast(robotModel.getLinkActors()->GetItemAsObject(i)));
	}

	// The shell/wrapper algorithm to use as a parameter to the motion-planning algorithm
	auto wrapper_algo = std::make_shared<StreamingConvexHull>(StreamingConvexHull::fromSpherifiedCube(4));

	double time = 0.0;

	// The "main loop" (interval callback) of the program.
	auto callback = [&]() {

		time += 0.01;

		// Update the robot's visualization to match the current state.

		moveit::core::RobotState state(robot);
		setStateToTrajectoryPoint(state, time, traj);
		robotModel.applyState(state);

		visualizerWindow->Render();

	};

	// Set our "main loop" callback to be called every frame.
	vtkNew<vtkFunctionalCallback> cb;
	cb->setEventId(vtkCommand::TimerEvent);
	cb->setCallback(callback);
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

	// Run the app until termination.
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;


}