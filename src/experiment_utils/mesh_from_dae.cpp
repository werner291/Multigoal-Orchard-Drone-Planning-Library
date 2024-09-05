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

	// Tell Assimp to delete everything but vertices and faces.
	importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
								aiComponent_NORMALS                  |
								aiComponent_TANGENTS_AND_BITANGENTS  |
								aiComponent_COLORS                   |
								aiComponent_TEXCOORDS                |
								aiComponent_BONEWEIGHTS              |
								aiComponent_ANIMATIONS               |
								aiComponent_TEXTURES                 |
								aiComponent_LIGHTS                   |
								aiComponent_CAMERAS                  |
								aiComponent_MATERIALS);

	const aiScene *scene = importer.ReadFile(dae_file, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_RemoveComponent);
	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
		throw std::runtime_error("Failed to load mesh from " + dae_file + ": " + importer.GetErrorString()
		+ " (Are you in the right working directory, and are the mesh files available in it?)");
	}

	Mesh mesh;

	// This is a tree mesh if it contains the word "fruit", "leaves", or "trunk"
	bool is_tree_mesh = dae_file.find("fruit") != std::string::npos ||
						dae_file.find("leaves") != std::string::npos ||
						dae_file.find("trunk") != std::string::npos;

	for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
		const aiMesh *ai_mesh = scene->mMeshes[i];
		size_t index_offset = mesh.vertices.size();
		for (unsigned int j = 0; j < ai_mesh->mNumVertices; ++j) {
			const aiVector3D &vertex = ai_mesh->mVertices[j];

			if (is_tree_mesh) {

				mesh.vertices.emplace_back(
						vertex.x / (100.0 / CENTIMETERS_PER_INCH), // There appears to me some sort of cm/mm conversion going on with DAE, hence the weird division
						vertex.z / (100.0 / CENTIMETERS_PER_INCH), // Z and Y are swapped intentionally
						vertex.y / (100.0 / CENTIMETERS_PER_INCH)
				);

			} else {
				mesh.vertices.emplace_back(
						vertex.x / 10.0,
						vertex.y / 10.0,
						vertex.z / 10.0
				);
			}
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
