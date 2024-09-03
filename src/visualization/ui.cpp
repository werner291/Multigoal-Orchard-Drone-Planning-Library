// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "ui.h"

#include <vtkTextProperty.h>
#include <vtkRenderer.h>

vtkSmartPointer<vtkTextActor>
mgodpl::visualization::add_text_label(SimpleVtkViewer &viewer,
									  const std::string &text,
									  double x,
									  double y,
									  int font_size) {
	vtkNew<vtkTextActor> text_actor;
	text_actor->SetInput(text.c_str());
	text_actor->SetPosition(x, y);
	text_actor->GetTextProperty()->SetFontSize(font_size);
	viewer.viewerRenderer->AddActor2D(text_actor);
	return text_actor;
}
