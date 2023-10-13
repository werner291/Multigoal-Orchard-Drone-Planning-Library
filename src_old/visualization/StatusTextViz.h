// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#ifndef NEW_PLANNERS_STATUSTEXTVIZ_H
#define NEW_PLANNERS_STATUSTEXTVIZ_H

#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkTextActor.h>

/**
 * A class that can be used to display a text in the bottm left corner of a VTK render window.
 */
class StatusTextViz {

	vtkNew<vtkTextActor> textActor; //< The actor that displays the text.

public:
	/**
	 * Create a new status text and add it to the given renderer.
	 *
	 * @param renderer 		The renderer to add the text to.
	 */
	StatusTextViz(vtkRenderer *renderer);

	/**
	 * Update the text that is displayed.
	 * @param text  The text to display.
	 */
	void updateText(const std::string &text);
};


#endif //NEW_PLANNERS_STATUSTEXTVIZ_H
