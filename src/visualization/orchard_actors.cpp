// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/enumerate.hpp>
#include "orchard_actors.h"
#include "../utilities/vtk.h"
#include "../apple_status_color_coding.h"
#include "../utilities/mesh_utils.h"

#include <vtkProperty.h>

/**
 * @brief Creates vtkActors from meshes representing a tree and its fruit and adds them to a SimpleVtkViewer.
 * @param meshes The meshes to create actors from.
 * @param apple_actors A vector to store the actors representing the apples.
 * @param viewer The viewer to add the actors to.
 */
std::vector<vtkActor *> createActors(const TreeMeshes &meshes, SimpleVtkViewer &viewer) {
	vtkNew<vtkActorCollection> actors;

	auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
	setColorsByEncoding(tree_actor, {0.5, 0.3, 0.1}, false);
	actors->AddItem(tree_actor);

	auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
	setColorsByEncoding(leaves_actor, {0.0, 0.9, 0.0}, false);
	actors->AddItem(leaves_actor);

	std::vector<vtkActor *> apple_actors;

	for (const auto &[fruit_index, fruit_mesh]: meshes.fruit_meshes | ranges::views::enumerate) {
		auto fruit_actor = createActorFromMesh(fruit_mesh);
		fruit_actor->GetProperty()->SetDiffuseColor(0.0, 0.0, 0.0);
		actors->AddItem(fruit_actor);
		apple_actors.push_back(fruit_actor);
	}

	{
		auto ground_actor = createActorFromMesh(createGroundPlane(10.0, 10.0));

		ground_actor->GetProperty()->SetDiffuseColor(0.5, 0.8, 0.5);

		actors->AddItem(ground_actor);
	}

	viewer.addActorCollection(actors);

	return apple_actors;
}
