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
#include "../planning/JointSpacePoint.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/VtkRobotModel.h"

using namespace mgodpl;
using namespace tree_meshes;
//using namespace visualization;
using namespace math;
using namespace moveit_facade;

vtkSmartPointer<vtkActor> mkPointMarkerSphere(math::Vec3d &target,
											  SimpleVtkViewer &viewer) {// Create a mapper and actor for the sphere.
	// Create a small sphere at the target point.
	vtkNew<vtkSphereSource> sphere;
	sphere->SetRadius(0.1);
	sphere->Update();

	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphere->GetOutputPort());
	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetColor(1, 0, 0);
	sphereActor->SetPosition(target.components.data());
	viewer.addActor(sphereActor);

	return sphereActor;
}

int main(int argc, char** argv) {

	const auto &robot = experiment_assets::loadRobotModel(1.0);

	SimpleVtkViewer viewer;
	viewer.lockCameraUp();
	Vec3d target(0, 0, 2);
	auto target_marker = mkPointMarkerSphere(target, viewer);
	const size_t N_SAMPLES = 20;

	for (int i = 0; i < N_SAMPLES; ++i) {
		JointSpacePoint jt = experiment_state_tools::randomUprightWithBase(*robot, 0.0, i);

		experiment_state_tools::moveEndEffectorNearPoint(
				*robot, jt, target, 0.
		);

		visualization::VtkRobotModel robotModelViz(robot, jt, {0.5, 0.5, 0.5});
		viewer.addActorCollection(robotModelViz.getLinkActors());
	}
	viewer.start();

}