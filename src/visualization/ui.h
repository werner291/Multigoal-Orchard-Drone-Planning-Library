// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-9-24.
//

#ifndef MGODPL_UI_H
#define MGODPL_UI_H

#include <vtkTextActor.h>
#include "SimpleVtkViewer.h"

namespace mgodpl::visualization {

	/**
	 * \brief Adds a text label to the VTK viewer.
	 *
	 * This function creates a text label and adds it to the viewer at the specified position.
	 *
	 * \param viewer The VTK viewer to which the text label will be added.
	 * \param text The text to be displayed.
	 * \param x The x-coordinate of the text label's position.
	 * \param y The y-coordinate of the text label's position.
	 * \param font_size The font size of the text label.
	 */
	vtkSmartPointer<vtkTextActor>
	add_text_label(SimpleVtkViewer &viewer, const std::string &text, double x, double y, int font_size = 24);
}

#endif //MGODPL_UI_H
