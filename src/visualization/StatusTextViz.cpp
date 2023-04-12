// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "StatusTextViz.h"

#include <vtkTextProperty.h>

StatusTextViz::StatusTextViz(vtkRenderer *renderer) {
	textActor->SetPosition2(10, 40);
	textActor->GetTextProperty()->SetFontSize(10);
	renderer->AddActor2D(textActor);
}

void StatusTextViz::updateText(const std::string &text) {
	textActor->SetInput(text.c_str());
	textActor->Modified();
}
