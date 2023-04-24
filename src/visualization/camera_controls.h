// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#ifndef NEW_PLANNERS_CAMERA_CONTROLS_H
#define NEW_PLANNERS_CAMERA_CONTROLS_H

class vtkRenderer;
class vtkRenderWindowInteractor;

/**
 * @brief Enforce camera up vector to (0,0,1) after interaction with a VTK widget.
 *
 * This function enforces the camera up vector to (0,0,1) after the user interacts with a VTK widget,
 * so that the user cannot accidentally change the view orientation. The function assumes that the
 * initial camera position is (10,0,3), the focal point is (0,0,2), and the initial camera up vector
 * is (0,0,1).
 *
 * @param renderer A reference to the vtkRenderer instance to be modified.
 * @param interactor A pointer to the vtkRenderWindowInteractor instance of the widget.
 */
void enforceCameraUp(vtkRenderer *renderer, vtkRenderWindowInteractor *interactor);

#endif //NEW_PLANNERS_CAMERA_CONTROLS_H
