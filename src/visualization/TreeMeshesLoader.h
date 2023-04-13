// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-4-23.
//

#ifndef NEW_PLANNERS_TREEMESHESLOADER_H
#define NEW_PLANNERS_TREEMESHESLOADER_H

#include <QObject>
#include <QFuture>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>
#include <utility>

#include "../TreeMeshes.h"

class TreeMeshesLoader : public QObject {
Q_OBJECT

public:
	explicit TreeMeshesLoader(QStringList sceneNames, QObject *parent = nullptr);

	void loadTreeMeshesAsync(int index);

signals:
	void treeMeshesLoaded(const TreeMeshes &meshes);

private slots:
	void onMeshesLoaded();

private:
	QStringList sceneNames;
	QFutureWatcher<TreeMeshes> meshes_watcher;
};

#endif //NEW_PLANNERS_TREEMESHESLOADER_H
