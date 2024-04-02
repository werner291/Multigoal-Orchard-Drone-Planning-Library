// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/18/24.
//

#include "mesh_from_dae.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

const double CENTIMETERS_PER_INCH = 2.54;

mgodpl::Mesh mgodpl::from_dae(const std::string &dae_file) {

	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(dae_file, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
		throw std::runtime_error("Failed to load mesh from " + dae_file + ": " + importer.GetErrorString());
	}

	Mesh mesh;

	// This is a tree mesh if it contains the word "fruit", "leaves", or "trunk"
	bool is_tree_mesh = dae_file.find("fruit") != std::string::npos ||
						dae_file.find("leaves") != std::string::npos ||
						dae_file.find("trunk") != std::string::npos;

	// We divide all coordinates by this due to a cm/mm conversion in the DAE files
	double MESH_SCALE_FACTOR = 10.0;

	// Something weird happened with a cm/inch conversion in the tree models, so we correct for that here
	if (is_tree_mesh) {
		MESH_SCALE_FACTOR *= CENTIMETERS_PER_INCH;
	}

	for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
		const aiMesh *ai_mesh = scene->mMeshes[i];
		size_t index_offset = mesh.vertices.size();
		for (unsigned int j = 0; j < ai_mesh->mNumVertices; ++j) {
			const aiVector3D &vertex = ai_mesh->mVertices[j];
			mesh.vertices.emplace_back(
					vertex.x/MESH_SCALE_FACTOR, // There appears to me some sort of cm/mm conversion going on with DAE, hence the division by 10
					vertex.z/MESH_SCALE_FACTOR, // Z and Y are swapped intentionally
					vertex.y/MESH_SCALE_FACTOR
									   );
		}
		for (unsigned int j = 0; j < ai_mesh->mNumFaces; ++j) {
			const aiFace &face = ai_mesh->mFaces[j];
			if (face.mNumIndices != 3) {
				throw std::runtime_error("Only triangular faces are supported");
			}
			mesh.triangles.push_back({face.mIndices[0] + index_offset,
									  face.mIndices[1] + index_offset,
									  face.mIndices[2] + index_offset});
		}
	}

	return mesh;

}
