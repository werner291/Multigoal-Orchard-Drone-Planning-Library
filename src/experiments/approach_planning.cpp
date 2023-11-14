// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vtkActor.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include "../experiment_utils/TreeMeshes.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../experiment_utils/load_robot_model.h"
#include "../planning/CollisionDetection.h"
#include "../planning/JointSpacePoint.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/VtkRobotModel.h"

using namespace mgodpl;
using namespace tree_meshes;
//using namespace visualization;
using namespace math;
using namespace moveit_facade;

vtkSmartPointer<vtkActor> mkPointMarkerSphere(const math::Vec3d& target,
                                              SimpleVtkViewer &viewer) {// Create a mapper and actor for the sphere.
	// Create a small sphere at the target point.
	vtkNew<vtkSphereSource> sphere;
	sphere->SetRadius(0.05);
	sphere->Update();

	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphere->GetOutputPort());
	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetColor(1, 0, 0);
	sphereActor->SetPosition(target.x(), target.y(), target.z());
	viewer.addActor(sphereActor);

	return sphereActor;
}

int main(int argc, char** argv) {

	const auto &robot = experiment_assets::loadRobotModel(1.0);

	const auto& treemodel = loadTreeMeshes("appletree");

	CollisionDetection cd({treemodel.trunk_mesh}, robot);

	const auto& fruit = computeFruitPositions(treemodel);

	const size_t N_SAMPLES = 10000;

	SimpleVtkViewer viewer;
	viewer.addMesh(treemodel.trunk_mesh, {0.5, 0.3, 0.1});

	viewer.lockCameraUp();
	Vec3d target(0, 0, 2);

	int least_successes = N_SAMPLES;
	math::Vec3d least_successes_fruit = fruit[0];

	for (const auto& f : fruit) {

		int successes = 0;

		for (int sample_i = 0; sample_i < N_SAMPLES; ++sample_i) {
			JointSpacePoint jt = experiment_state_tools::genGoalSampleUniform(f, sample_i, *robot);
			if (!cd.collides(jt))
			{
				++successes;
			}
		}

		if (successes < least_successes) {
			least_successes = successes;
			least_successes_fruit = f;
		}

		double success_rate = successes / (double)N_SAMPLES;

		auto target_marker = mkPointMarkerSphere(f, viewer);

		target_marker->GetProperty()->SetColor(
			1.0 - success_rate,
			success_rate,
			0.0
		);

	}

	std::cout << "Target with least successes: " << least_successes_fruit << " with " << least_successes << " successes." << std::endl;

	// For the sample with the least successes, visualize the robot.
	for (int sample_i = 0; sample_i < 100; ++sample_i) {
		JointSpacePoint jt = experiment_state_tools::genGoalSampleUniform(least_successes_fruit, sample_i, *robot);


			visualization::VtkRobotModel robotModelViz(robot, jt, !cd.collides(jt) ? Vec3d(0.5, 0.5, 0.5) : Vec3d(0.5, 0.0, 0.0));

			viewer.addActorCollection(robotModelViz.getLinkActors());
	}

	viewer.setCameraTransform(
		least_successes_fruit + Vec3d(1, 0, 0),
		least_successes_fruit);

	auto target_marker = mkPointMarkerSphere(target, viewer);
	viewer.start();

}