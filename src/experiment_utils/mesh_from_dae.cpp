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

mgodpl::Mesh mgodpl::from_dae(const std::string &dae_file) {

	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(dae_file, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
		throw std::runtime_error("Failed to load mesh from " + dae_file + ": " + importer.GetErrorString());
	}

	Mesh mesh;

	for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
		const aiMesh *ai_mesh = scene->mMeshes[i];
		for (unsigned int j = 0; j < ai_mesh->mNumVertices; ++j) {
			const aiVector3D &vertex = ai_mesh->mVertices[j];
			mesh.vertices.emplace_back(vertex.x, vertex.y, vertex.z);
		}
		for (unsigned int j = 0; j < ai_mesh->mNumFaces; ++j) {
			const aiFace &face = ai_mesh->mFaces[j];
			if (face.mNumIndices != 3) {
				throw std::runtime_error("Only triangular faces are supported");
			}
			mesh.triangles.push_back({face.mIndices[0], face.mIndices[1], face.mIndices[2]});
		}

		break; // Only one mesh is supported
	}

	return mesh;

}
