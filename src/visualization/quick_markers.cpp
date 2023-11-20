//
// Created by werner on 17-11-23.
//

#include "quick_markers.h"

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

#include "SimpleVtkViewer.h"

namespace mgodpl::visualization
{
    vtkSmartPointer<vtkActor> mkPointMarkerSphere(const mgodpl::math::Vec3d& target,
                                                  mgodpl::SimpleVtkViewer& viewer)
    {
        // Create a mapper and actor for the sphere.
        // Create a small sphere at the target point.
        vtkNew<vtkSphereSource> sphere;
        sphere->SetRadius(0.05);
        sphere->Update();

        vtkNew<vtkPolyDataMapper> sphereMapper;
        sphereMapper->SetInputConnection(sphere->GetOutputPort());
        vtkNew<vtkActor> sphereActor;
        sphereActor->SetMapper(sphereMapper);
        sphereActor->GetProperty()->SetColor(1, 0, 0);
        sphereActor->SetPosition(target.x(), target.y(), target.z());
        viewer.addActor(sphereActor);

        return sphereActor;
    }
}
