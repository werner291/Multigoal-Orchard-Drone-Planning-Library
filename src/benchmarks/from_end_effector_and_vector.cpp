// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/17/24.
//

#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/tree_benchmark_data.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/state_tools.h"
#include "../visualization/robot_state.h"
#include "../visualization/RunQueue.h"
#include "../visualization/Throttle.h"
#include "../visualization/declarative.h"

#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkArrowSource.h>
#include <vtkMatrix4x4.h>

#include "../visualization/VtkLineSegmentVizualization.h"

using namespace mgodpl;

import visualization.ThrottledRunQueue;

/**
 * \brief Picks a random surface point on the given convex hull.
 *
 * This function selects a random face on the convex hull and generates random
 * barycentric coordinates within that face. The barycentric coordinates are
 * then normalized to ensure they sum to 1.
 *
 * \param rng A random number generator.
 * \param convex_hull The convex hull data from which to pick a random surface point.
 * \return A Face_location object containing the face ID and barycentric coordinates.
 */
cgal::Surface_mesh_shortest_path::Face_location pickRandomSurfacePoint(random_numbers::RandomNumberGenerator &rng,
                                                                       const cgal::CgalMeshData &convex_hull) {
	auto face_id = rng.uniformInteger(0, convex_hull.convex_hull.number_of_faces() - 1);
	std::array<double, 3> barycentric_coords = {
		rng.uniformReal(0.0, 1.0), rng.uniformReal(0.0, 1.0), rng.uniformReal(0.0, 1.0)
	};
	// Normalize the barycentric coordinates
	double sum = barycentric_coords[0] + barycentric_coords[1] + barycentric_coords[2];
	barycentric_coords[0] /= sum;
	barycentric_coords[1] /= sum;
	barycentric_coords[2] /= sum;

	return cgal::Surface_mesh_shortest_path::Face_location{face_id, barycentric_coords};
}


REGISTER_VISUALIZATION(from_end_effector_and_vector) {
	auto robot = experiments::createStraightArmRobotModel(1.0);
	auto tree = experiments::loadBenchmarkTreemodelData("appletree");

	// Visualize the tree:
	viewer.addTree(tree.tree_mesh, true, true);

	// Visualize the convex hull:
	mgodpl::visualization::visualize(viewer, *tree.tree_convex_hull);

	random_numbers::RandomNumberGenerator rng(42);

	mgodpl::visualization::ThrottledRunQueue run_queue_throttled;

	std::thread script([&]() {
		auto segments = run_queue_throttled.run_main<std::unique_ptr<VtkLineSegmentsVisualization> >(
			[&](SimpleVtkViewer &viewer) {
				auto segs = std::make_unique<VtkLineSegmentsVisualization>(1, 0, 1);
				viewer.addActor(segs->getActor());
				return segs;
			});

		while (true) {
			auto surface_pt = pickRandomSurfacePoint(rng, *tree.tree_convex_hull);
			auto surface_data = cgal::from_face_location(surface_pt, *tree.tree_convex_hull);

			run_queue_throttled.run_main_void(
				[&](SimpleVtkViewer &viewer) {
					math::Vec3d camera_vector = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), M_PI * 0.4).
							rotate(
								surface_data.normal * 5.0);
					camera_vector.setZ(1.0);

					viewer.setCameraTransform(
						surface_data.surface_point + camera_vector,
						surface_data.surface_point);
				});

			run_queue_throttled.run_main_void(
				[&](SimpleVtkViewer &viewer) {
					vtkNew<vtkArrowSource> arrow;
					arrow->SetTipLength(0.2);

					vtkNew<vtkPolyDataMapper> mapper;
					mapper->SetInputConnection(arrow->GetOutputPort());
					vtkNew<vtkActor> actor;
					actor->SetMapper(mapper);

					// Build a 3x3 rotation matrix, with the normal as the first row.
					math::Vec3d x_axis = surface_data.normal.cross(math::Vec3d::UnitX()).normalized();
					math::Vec3d y_axis = surface_data.normal.cross(x_axis).normalized();
					vtkNew<vtkMatrix4x4> matrix;
					matrix->Identity();
					for (int i = 0; i < 3; ++i) {
						matrix->SetElement(i, 0, surface_data.normal[i] * 2.0);
						matrix->SetElement(i, 1, x_axis[i] * 0.5);
						matrix->SetElement(i, 2, y_axis[i] * 0.5);
					}

					// Work in the position:
					matrix->SetElement(0, 3, surface_data.surface_point.x());
					matrix->SetElement(1, 3, surface_data.surface_point.y());
					matrix->SetElement(2, 3, surface_data.surface_point.z());

					actor->SetUserMatrix(matrix);

					actor->GetProperty()->SetColor(1, 0, 1);

					viewer.addActor(actor);
				});

			run_queue_throttled.throttle.wait_and_advance(30);

			auto actors = run_queue_throttled.run_main<vizualisation::RobotActors>(
				[&](SimpleVtkViewer &viewer) {
					const RobotState st = fromEndEffectorAndVector(robot,
					                                               surface_data.surface_point,
					                                               surface_data.normal);
					auto actors = vizualisation::vizualize_robot_state(viewer, robot, st);
					for (auto &a: actors.actors) {
						a->GetProperty()->SetOpacity(0.5);
					}
					return actors;
				});

			run_queue_throttled.throttle.wait_and_advance(30);

			break;

			run_queue_throttled.run_main_void([&](SimpleVtkViewer &viewer) {
				for (auto &a: actors.actors) {
					viewer.removeActor(a.Get());
				}
			});
		}
	});

	viewer.addTimerCallback([&]() {
		run_queue_throttled.run_queue.run_all(viewer);
		run_queue_throttled.throttle.allow_advance();
	});

	viewer.lockCameraUp();

	viewer.start();
}
