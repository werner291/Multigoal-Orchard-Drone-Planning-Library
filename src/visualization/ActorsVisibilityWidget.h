// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-4-23.
//

#ifndef NEW_PLANNERS_ACTORSVISIBILITYWIDGET_H
#define NEW_PLANNERS_ACTORSVISIBILITYWIDGET_H


#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>

class ActorsVisibilityWidget : public QWidget {
Q_OBJECT

public:
	explicit ActorsVisibilityWidget(vtkRenderWindow *renderWindow, QWidget *parent = nullptr);

public slots:
	void setActorsWithLabels(const std::vector<std::pair<std::vector<vtkSmartPointer<vtkActor>>, std::string>> &actors_with_labels);

private:
	QVBoxLayout *layout;
	vtkRenderWindow *renderWindow;
	std::vector<QCheckBox *> checkBoxes;
};



#endif //NEW_PLANNERS_ACTORSVISIBILITYWIDGET_H
