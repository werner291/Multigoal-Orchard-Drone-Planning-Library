
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPointData.h>
#include <vtkCallbackCommand.h>

#include <utility>

#include <range/v3/view/enumerate.hpp>

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <geometric_shapes/shape_operations.h>

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../exploration/StupidAlgorithm.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/vtk.h"
#include "../vtk/SimulatedSensor.h"
#include "../vtk/Viewer.h"
#include "../utilities/msgs_utilities.h"
#include "../StreamingConvexHull.h"

#include "../exploration/DynamicMeshHullAlgorithm.h"
#include "../WorkspaceSpec.h"
#include "../CurrentPathState.h"
#include "../utilities/moveit.h"
#include "../vtk/VisualizationSpecifics.h"
#include "../utilities/convex_hull.h"
#include "../utilities/math_utils.h"
#include "../utilities/mesh_utils.h"

/**
 * Given a SimplifiedOrchard, create a SegmentedPointCloud that gives an initial hint about the contents
 * of the workspace, meant to be given initially to the exploration algorithm.
 *
 * It can be more or less detailed depending on how difficult you want the exploration to be.
 *
 * @param orchard 		The orchard to create the initial hint for.
 * @return 				The SegmentedPointCloud.
 */
SegmentedPointCloud generateInitialCloud(const SimplifiedOrchard &orchard);

/**
 * Build a MoveIt collision environment from the given workspace specification.
 *
 * @param spec 			The workspace specification.
 *
 * @return 				The collision environment.
 */
std::unique_ptr<collision_detection::CollisionEnvFCL> buildOrchardAndRobotFCLCollisionEnvironment(const WorkspaceSpec &spec);

WorkspaceSpec buildWorkspaceSpec() {
	auto drone = loadRobotModel();

	auto initial_state = mkInitialState(drone);

	setBaseOrientation(initial_state, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ())));

	initial_state.update(true);

	return std::move<WorkspaceSpec>({
			drone,
			initial_state,
			{{{ { 0.0, 0.0 }, loadTreeMeshes("appletree") }}}
	});
}

int main(int, char*[]) {

	WorkspaceSpec workspaceSpec = buildWorkspaceSpec();

	auto collision_env = buildOrchardAndRobotFCLCollisionEnvironment(workspaceSpec);

	auto surface_points = buildScanTargetPoints(workspaceSpec.orchard.trees[0].second.fruit_mesh, 2000);

	ScannablePointsIndex scannablePointsIndex(surface_points);

	FruitSurfaceScanTargetsActor fruitSurfaceScanTargetsActor(surface_points);

	VtkRobotmodel robotModel(workspaceSpec.robotModel,workspaceSpec.initialState);

	SimulatedSensor sensor = buildSensorSimulator(workspaceSpec.orchard, robotModel);

	vtkNew<vtkActor> pointCloudActor = buildDepthImagePointCloudActor(sensor.getPointCloudOutputPort());

	ConvexHullActor convexHullActor;

	vtkNew<vtkPolyData> targetPointData;
	vtkNew<vtkPolyDataMapper> targetPointMapper;
	vtkNew<vtkActor> targetPointActor;

	targetPointMapper->SetInputData(targetPointData);
	targetPointActor->SetMapper(targetPointMapper);
	targetPointActor->GetProperty()->SetColor(1, 0, 0);
	targetPointActor->GetProperty()->SetPointSize(10);
	targetPointActor->GetProperty()->SetLineWidth(5);

	VtkPolyLineVizualization visitOrderVisualization(1.0,0.5,0.5);

	VtkPolyLineVizualization ee_trace_visualization(0.0,0.0,1.0);

	Viewer viewer = buildViewer(workspaceSpec.orchard,robotModel,
								{fruitSurfaceScanTargetsActor.fruitSurfacePointsActor,
								 visitOrderVisualization.getActor(),
								 ee_trace_visualization.getActor(),
								 targetPointActor,
								 convexHullActor.actor,
								 pointCloudActor,
								}
	);

	CurrentPathState currentPathState(workspaceSpec.initialState);

	auto wrapper_algo = std::make_shared<StreamingConvexHull>(StreamingConvexHull::fromSpherifiedCube(4));

	DynamicMeshHullAlgorithm dbsa(workspaceSpec.initialState, [&](robot_trajectory::RobotTrajectory trajectory) {
		currentPathState.newPath(trajectory);

		{
			std::vector<Eigen::Vector3d> points;

			for (size_t wp_idx = 0; wp_idx < trajectory.getWayPointCount(); wp_idx++) {
				points.push_back(trajectory.getWayPoint(wp_idx).getGlobalLinkTransform("end_effector").translation());
			}

			ee_trace_visualization.updateLine(points);
		}

	}, std::move(wrapper_algo));

	dbsa.updatePointCloud(currentPathState.current_state, generateInitialCloud(workspaceSpec.orchard));

//	size_t countdown = 300;

	auto callback = [&]() {

//		if (countdown > 0) {
//			countdown--;
//		} else {
			currentPathState.advance(0.01);
//		}


		if (checkCollision(currentPathState.getCurrentState(), *collision_env)) {
			std::cout << "Oh no!" << std::endl;
		}

		robotModel.applyState(currentPathState.getCurrentState());

		Eigen::Isometry3d eePose = currentPathState.getCurrentState().getGlobalLinkTransform("end_effector");
		sensor.requestRender(eePose);

		// Nullptr check to avoid a race condition where the callback is called before the first point cloud is rendered.
		// FIXME: Ideally, I'd like to make sure the algorithm sees the point cloud exactly once.
		if (auto points = sensor.extractLatestPointCloud()) {

			dbsa.updatePointCloud(currentPathState.getCurrentState(), *points);

			{
				vtkNew<vtkPoints> pointsVtk;
				vtkNew<vtkCellArray> cells;
				for (const auto &[original, projected]: dbsa.getTargetPointsOnChullSurface()) {
					cells->InsertNextCell(2);
					cells->InsertCellPoint(pointsVtk->InsertNextPoint(projected.data()));
					cells->InsertCellPoint(pointsVtk->InsertNextPoint(original.data()));
				}
				targetPointData->SetPoints(pointsVtk);
				targetPointData->SetLines(cells);
				targetPointData->Modified();

				convexHullActor.update(dbsa.getConvexHull()->toMesh());

			}

			{
				std::vector<Eigen::Vector3d> visit_order_points {
					eePose.translation()
				};
				for (const auto &point: dbsa.getVisitOrdering().getVisitOrdering()) {
					visit_order_points.push_back(dbsa.getTargetPointsOnChullSurface()[point].hull_location);
				}
				visitOrderVisualization.updateLine(visit_order_points);
			}


			auto scanned_points = scannablePointsIndex
					.findScannedPoints(eePose.translation(),
									   eePose.rotation() * Eigen::Vector3d(0, 1, 0),
									   M_PI / 4.0,
									   0.1,
									   *points);

			fruitSurfaceScanTargetsActor.markAsScanned(scanned_points);
		}

		viewer.requestRender();

	};

	viewer.setIntervalCallback(callback);

	viewer.start();

	return EXIT_SUCCESS;
}

std::unique_ptr<collision_detection::CollisionEnvFCL> buildOrchardAndRobotFCLCollisionEnvironment(const WorkspaceSpec &workspaceSpec) {
	auto collision_world = std::make_shared<collision_detection::World>();

	for (const auto& [tree_id, tree] : workspaceSpec.orchard.trees | ranges::views::enumerate) {
		Eigen::Isometry3d tree_pose = Eigen::Isometry3d::Identity();
		tree_pose.translation() = Eigen::Vector3d(tree.first.x(), tree.first.y(), 0);
		collision_world->addToObject("tree_" + std::to_string(tree_id) + "_trunk",
									 shapes::ShapeConstPtr(shapes::constructShapeFromMsg(tree.second.trunk_mesh)),
									 tree_pose);
	}

	return std::move(std::make_unique<collision_detection::CollisionEnvFCL>(workspaceSpec.robotModel, collision_world));
}

SegmentedPointCloud generateInitialCloud(const SimplifiedOrchard &orchard) {// Pick a random fruit mesh vertex.
	ompl::RNG rng;
	auto fruitMeshVertex = orchard.trees[0].second.fruit_mesh.vertices[rng.uniformInt(0, orchard.trees[0].second.fruit_mesh.vertices.size() - 1)];

	SegmentedPointCloud initialCloud {
			{SegmentedPointCloud::Point{SegmentedPointCloud::PT_TARGET,
										{fruitMeshVertex.x, fruitMeshVertex.y, fruitMeshVertex.z},

			}}};
	return initialCloud;
}





