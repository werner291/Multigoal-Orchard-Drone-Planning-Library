// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <range/v3/view/zip.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/range/conversion.hpp>

#include "shell_visualization.h"

vtkSmartPointer<vtkActor> mkSphereShellActor(const WorkspaceSphereShell &sphereshell) {
	vtkNew<vtkSphereSource> sphereSource;
	sphereSource->SetCenter(sphereshell.getCenter().x(), sphereshell.getCenter().y(), sphereshell.getCenter().z());
	sphereSource->SetRadius(sphereshell.getRadius());
	sphereSource->SetThetaResolution(32);
	sphereSource->SetPhiResolution(16);

	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);

	sphereActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
	sphereActor->GetProperty()->SetOpacity(0.5);

	return sphereActor;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeOneToAllIdealizedPathEdges(const std::vector<Apple> &apples,
								  int apple_i,
								  const std::function<std::vector<Eigen::Vector3d>(const Eigen::Vector3d &,
																				   const Eigen::Vector3d &)> &path_fn) {

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

	for (size_t apple_j = 0; apple_j < apples.size(); apple_j++) {

		// Get the range of points for the shell between the two apples
		std::vector<Eigen::Vector3d> shell_points;

		shell_points = path_fn(apples[apple_i].center, apples[apple_j].center);

		// Convert the range of points to a vector of pairs of points
		auto edges_for_path =
				ranges::views::zip(shell_points, shell_points | ranges::views::drop(1)) | ranges::to_vector;

		// Add the edges to the vector of edges
		edges.insert(edges.end(), edges_for_path.begin(), edges_for_path.end());
	}

	return edges;
}
