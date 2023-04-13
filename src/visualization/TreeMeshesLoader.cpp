// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-4-23.
//

#include "TreeMeshesLoader.h"

TreeMeshesLoader::TreeMeshesLoader(QStringList sceneNames, QObject *parent) : QObject(parent), sceneNames(std::move(sceneNames)) {}

void TreeMeshesLoader::loadTreeMeshesAsync(int index) {

}

void TreeMeshesLoader::onMeshesLoaded() {
	emit treeMeshesLoaded(meshes_watcher.result());
}
