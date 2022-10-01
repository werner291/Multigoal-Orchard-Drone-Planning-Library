
#ifndef NEW_PLANNERS_VTK_H
#define NEW_PLANNERS_VTK_H

#include <vtkPolyDataMapper.h>
#include <moveit/robot_model/link_model.h>

vtkNew<vtkPolyDataMapper> polyDataForLink(const moveit::core::LinkModel *lm);

#endif //NEW_PLANNERS_VTK_H
