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

using namespace mgodpl;
using namespace tree_meshes;
using namespace visualization;
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

	math::Vec3d target(0,0,2);

	const auto &robot = experiment_assets::loadRobotModel(1.0);

	SimpleVtkViewer viewer;

	auto target_marker = mkPointMarkerSphere(target, viewer);

	//	std::vector<TreeMeshes> meshes = mgodpl::tree_meshes::loadAllTreeModels(std::numeric_limits<int>::max(),
//																			std::numeric_limits<int>::max());
//
//

	// Take 50 goal samples.

	std::vector<JointSpacePoint> samples;

	for (int i = 0; i < 50; ++i) {
		samples.push_back(experiment_state_tools::randomUprightWithBase(*robot, 0.0, i));

		experiment_state_tools::
	}

	viewer.start();
}