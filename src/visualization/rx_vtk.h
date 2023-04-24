
#include <rxcpp/rx-observable.hpp>

#include "LabeledActors.h"

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>

void updateRendererContents(rxcpp::observable<LabeledActors> actorVectors,
                            vtkSmartPointer<vtkRenderWindow> renderWindow,
                            QVTKOpenGLNativeWidget *vtkWidget);

void addReactiveActor(rxcpp::observable<vtkSmartPointer<vtkActor>> actor_rx,
                      vtkSmartPointer<vtkRenderWindow> renderWindow);

class WorkspaceSphereShell;
class CuttingPlaneConvexHullShell;
class CGALMeshShell;
struct ShellWrapper;

rxcpp::observable<ShellWrapper> createShellWrapperObservable(
    const rxcpp::observable<std::shared_ptr<WorkspaceSphereShell>>& current_sphere_shell,
    const rxcpp::observable<std::shared_ptr<CuttingPlaneConvexHullShell>>& cutting_plane_chull_shell,
    const rxcpp::observable<std::shared_ptr<CGALMeshShell>>& cgal_mesh_shell,
    const rxcpp::observable<int>& current_shell_index);