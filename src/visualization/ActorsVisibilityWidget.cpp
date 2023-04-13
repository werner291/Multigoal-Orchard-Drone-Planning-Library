// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-4-23.
//

#include "ActorsVisibilityWidget.h"

ActorsVisibilityWidget::ActorsVisibilityWidget(vtkRenderWindow *renderWindow, QWidget *parent) : QWidget(parent), renderWindow(renderWindow) {
	layout = new QVBoxLayout(this);
}

void
ActorsVisibilityWidget::setActorsWithLabels(const std::vector<std::pair<std::vector<vtkSmartPointer<vtkActor>>, std::string>> &actors_with_labels) {


	std::cout << "ActorsVisibilityWidget::setActorsWithLabels" << std::endl;

	// Clear the existing checkboxes and layout items
	for (auto &checkBox : checkBoxes) {
		layout->removeWidget(checkBox);
		delete checkBox;
	}
	checkBoxes.clear();

	for (auto [actors, label] : actors_with_labels) {

		std::cout << "Making checkbox for " << label << " with pointer " << (long) actors.front().GetPointer() << std::endl;

		auto checkBox = new QCheckBox(QString(label.c_str()));
		checkBox->setChecked(actors.front()->GetVisibility());

		std::vector<vtkSmartPointer<vtkActor>> actors_copy = actors; // Copy the actors vector to a new vector, because the original vector will be destroyed when this function returns

		QObject::connect(checkBox, &QCheckBox::stateChanged,
						 [actors_copy, renderWindow = this->renderWindow](int state) {
			std::cout << "ActorsVisibilityWidget::setActorsWithLabels::stateChanged" << std::endl;
							 for (auto &actor : actors_copy) {
								 actor->SetVisibility(state == Qt::Checked);
								 std::cout << "Is visible: " << actor->GetVisibility() << " ( " << (long) actor.GetPointer() << " )" << std::endl;
								 actor->Modified();
							 }
							 renderWindow->Render();
						 });

		layout->addWidget(checkBox);
		checkBoxes.push_back(checkBox);
	}
}
