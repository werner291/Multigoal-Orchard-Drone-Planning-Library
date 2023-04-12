// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#include "VideoRecorder.h"

VideoRecorder::VideoRecorder(vtkRenderWindow *renderWindow, const std::string &filename) {

	// Make sure it's a .ogv file name
	if (filename.substr(filename.size() - 4) != ".ogv") {
		throw std::runtime_error("Cannot export video: filename must end with .ogv");
	}

	// Setup filter
	imageFilter->SetInput(renderWindow);
	imageFilter->SetInputBufferTypeToRGB();
	imageFilter->ReadFrontBufferOff();

	// Setup movie writer
	moviewriter->SetInputConnection(imageFilter->GetOutputPort());
	moviewriter->SetFileName(filename.c_str());
	moviewriter->Start();
}

void VideoRecorder::exportFrame() {
	if (finished) {
		throw std::runtime_error("Cannot export frame: movie export has already been finished.");
	}

	imageFilter->Update();
	imageFilter->Modified();
	moviewriter->Write();
}

void VideoRecorder::finish() {
	if (finished) {
		return;
	}

	moviewriter->End();
	finished = true;
}
