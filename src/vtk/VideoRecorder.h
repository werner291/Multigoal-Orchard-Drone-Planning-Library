// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#ifndef NEW_PLANNERS_VIDEORECORDER_H
#define NEW_PLANNERS_VIDEORECORDER_H

#include <vtkNew.h>
#include <vtkWindowToImageFilter.h>
#include <vtkOggTheoraWriter.h>
#include <vtkRenderWindow.h>

/**
 * A class that can be used to record a video of a VTK render window and save it to a file.
 */
class VideoRecorder {

	vtkNew<vtkWindowToImageFilter> imageFilter; //< The filter that converts the render window to an image.
	vtkNew<vtkOggTheoraWriter> moviewriter; //< The writer that writes the image to a file.
	bool finished = false; //< Whether the video has been finished.

public:
	/**
	 * Create a new video recorder and start recording.
	 *
	 * @param renderWindow 		The render window to record.
	 * @param filename 			The filename to save the video to.
	 */
	VideoRecorder(vtkRenderWindow *renderWindow, const std::string &filename);

	/**
	 * Export the current frame to the video; typically called after each render.
	 *
	 * Will throw an exception if finish() has already been called.
	 */
	void exportFrame();

	/**
	 * Finish the video and save it to the file.
	 */
	void finish();

};

#endif //NEW_PLANNERS_VIDEORECORDER_H
