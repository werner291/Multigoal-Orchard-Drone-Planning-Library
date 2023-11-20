//
// Created by werner on 17-11-23.
//

#ifndef QUICK_MARKERS_H
#define QUICK_MARKERS_H

#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include "../math/Vec3.h"

namespace mgodpl
{
    class SimpleVtkViewer;
}

namespace mgodpl::visualization
{
    vtkSmartPointer<vtkActor> mkPointMarkerSphere(const mgodpl::math::Vec3d& target,
                                                  mgodpl::SimpleVtkViewer& viewer);
}

#endif //QUICK_MARKERS_H
