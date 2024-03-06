// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-2-24.
//

#ifndef VISUALIZATION_FUNCTION_MACROS_H
#define VISUALIZATION_FUNCTION_MACROS_H

#include <functional>
#include <map>
#include <string>
#include <filesystem>

#include "SimpleVtkViewer.h"

// A top-level function that can be called to visualize something.
using VisFn = std::function<void(mgodpl::SimpleVtkViewer& viewer)>;

// A static map that maps a name to a visualization function.
extern std::map<std::string, VisFn> visualizations;

#define REGISTER_VISUALIZATION(name) \
    void name(mgodpl::SimpleVtkViewer& viewer); \
    static bool is_##name##_registered = [](){ \
    visualizations[#name] = name; \
    return true; \
    }(); \
    void name(mgodpl::SimpleVtkViewer& viewer)

#define CREATE_SLIDER(widget, rep, minValue, maxValue, initValue, title, posY) \
    vtkNew<vtkSliderRepresentation2D> rep##_rep; \
    rep##_rep->SetMinimumValue(minValue); \
    rep##_rep->SetMaximumValue(maxValue); \
    rep##_rep->SetValue(initValue); \
    rep##_rep->SetTitleText(title); \
    rep##_rep->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay(); \
    rep##_rep->GetPoint1Coordinate()->SetValue(0.05, posY); \
    rep##_rep->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay(); \
    rep##_rep->GetPoint2Coordinate()->SetValue(0.25, posY);                     \
    rep##_rep->GetTitleProperty()->SetColor(0,0,0);\
    vtkNew<vtkSliderWidget> widget; \
    widget->SetInteractor(viewer.renderWindowInteractor); \
    widget->SetRepresentation(rep##_rep); \
    widget->EnabledOn();

#endif //VISUALIZATION_FUNCTION_MACROS_H
