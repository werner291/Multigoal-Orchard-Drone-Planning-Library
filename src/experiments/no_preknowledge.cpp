// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

#include <vtkProperty.h>
#include <unordered_set>

#include "../experiment_utils/load_robot_model.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkRobotModel.h"
#include "../planning/moveit_state_tools.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../planning/RobotAlgorithm.h"
#include "../planning/BlindlyMoveToNextFruit.h"
#include "../experiment_utils/mesh_utils.h"
#include "../planning/CollisionDetection.h"
#include "../visualization/voxels.h"
#include "../math/AABBGrid.h"
#include "../math/Vec3.h"
#include "../visibility/GridVec.h"
#include "../math/Triangle.h"
#include "../visibility/voxel_visibility.h"
#include "../experiment_utils/VoxelShroudedSceneInfo.h"
#include "../visualization/VtkEndAndBaseTraceVisualization.h"
#include "../planning/JointSpacePath.h"
#include "../experiment_utils/VoxelShroudedSimulation.h"

static const double STEP_SIZE = 0.1;

static const double FRUIT_VISIT_DISTANCE_THRESHOLD = 0.15;

void updateTrace(const moveit::core::RobotModelPtr &robot,
				 mgodpl::moveit_facade::JointSpacePoint &current_state,
				 const std::shared_ptr<mgodpl::planning::BlindlyMoveToNextFruit> &algorithm,
				 mgodpl::visualization::VtkEndAndBaseTraceVisualization &trace_visualization) {

	mgodpl::moveit_facade::JointSpacePath interpolated_path;

	for (int segment = 0; segment < algorithm->plan.size(); ++segment) {
		const auto &a = segment == 0 ? current_state : algorithm->plan[segment - 1].point;
		const auto &b = algorithm->plan[segment].point;

		for (int i = 0; i < 20; ++i) {
			interpolated_path.path.push_back(interpolate(*robot, a, b, i / 20.0));
		}
	}

	trace_visualization.updateLine(interpolated_path);
}


using namespace mgodpl;
using namespace visualization;
using namespace experiment_state_tools;
using namespace moveit_facade;
using namespace planning;
using namespace math;

// hash for unordered map of vec3d
namespace std {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedStructInspection" // Clang is wrong; I use this in the unordered_map of actors.
	template<>
	struct hash<Vec3d> {
		size_t operator()(const Vec3d &v) const {
			return std::hash<double>()(v.x()) ^ std::hash<double>()(v.y()) ^ std::hash<double>()(v.z());
		}
	};
#pragma clang diagnostic pop
}

int main() {

	// TODO: Reminder, write down the idea of the snake path!

	const auto &robot = experiment_assets::loadRobotModel(1.0);

	const auto &tree_model = tree_meshes::loadTreeMeshes("appletree");

	// TODO test the determinism.
	mgodpl::simulation::VoxelShroudedSimulation simulation(robot, tree_model, 0, STEP_SIZE);

	SimpleVtkViewer viewer;
	viewer.lockCameraUp();

	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1});
	viewer.addMesh(tree_model.leaves_mesh, {0.1, 0.5, 0.1});
	viewer.addMesh(createGroundPlane(5.0, 5.0), {0.3, 0.5, 0.1});

	// Create an actor for every fruit:
	std::unordered_map<math::Vec3d, vtkActor *> fruit_actors;

	for (size_t i = 0; i < simulation.fruit_positions.size(); i++) {
		fruit_actors[simulation.fruit_positions[i]] = viewer.addMesh(tree_model.fruit_meshes[i], {0.5, 0.1, 0.1}).Get();
	}

	visualization::VtkRobotModel robotModelViz(robot, simulation.current_state, {0.5, 0.5, 0.5});
	viewer.addActorCollection(robotModelViz.getLinkActors());

	simulation.algorithm->on_target_rejected = [&](const Vec3d &fruit) {
		fruit_actors[fruit]->GetProperty()->SetColor(0.0, 0.0, 0.0);
	};

	simulation.algorithm->on_target_reached = [&](const Vec3d &fruit) {
		fruit_actors[fruit]->GetProperty()->SetColor(1.0, 1.0, 0.0);
	};

	VtkVoxelGrid voxel_grid(simulation.grid_coords, simulation.seen_space, {0.3, 0.3, 0.3}, true);
	viewer.addActor(voxel_grid.getActor());

	VtkEndAndBaseTraceVisualization trace_visualization(robot);

	updateTrace(robot, simulation.current_state, simulation.algorithm, trace_visualization);

	viewer.addActor(trace_visualization.barTrace.getActor());
	viewer.addActor(trace_visualization.endTrace.getActor());
	viewer.addActor(trace_visualization.baseTrace.getActor());

	viewer.addTimerCallback([&]() {

		if (!simulation.is_done() && !simulation.hasCollided) {
			simulation.update();

			voxel_grid.update(simulation.seen_space);

			robotModelViz.applyState(simulation.current_state);

			updateTrace(robot, simulation.current_state, simulation.algorithm, trace_visualization);

		}

	});

	//	viewer.startRecording("no_preknowledge.ogv");

	viewer.start();

	return 0;

}
