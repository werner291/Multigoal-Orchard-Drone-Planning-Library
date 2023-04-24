
#include "rx_vtk.h"

#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkActor.h>

#include "shell_visualization.h"

namespace Rx
{
    using namespace rxcpp;
    using namespace rxcpp::sources;
    using namespace rxcpp::operators;
    using namespace rxcpp::util;
}

#include <rxcpp/rx.hpp>

void updateRendererContents(rxcpp::observable<LabeledActors> actorVectors,
                            vtkSmartPointer<vtkRenderWindow> renderWindow,
                            QVTKOpenGLNativeWidget *vtkWidget)
{

    auto toClear = std::make_shared<std::vector<vtkActor *>>();
    actorVectors.subscribe([renderWindow, toClear, vtkWidget](const auto &actors)
                           {
        auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

        for (auto actor: *toClear) {
            renderer->RemoveActor(actor);
        }


        for (const auto &[actors, label]: actors) {
            for (const auto &actor: actors) {
                renderer->AddActor(actor);
                toClear->push_back(actor);
            }
        }

        vtkWidget->update();
        renderWindow->Render(); });
}

void addReactiveActor(rxcpp::observable<vtkSmartPointer<vtkActor>> actor_rx,
                      vtkSmartPointer<vtkRenderWindow> renderWindow)
{

    // Put that actor into the renderer as an actor, keeping track of the old actor so we can remove it.
    auto toClearshell = std::make_shared<vtkSmartPointer<vtkActor>>();

    actor_rx.subscribe([renderWindow = renderWindow.Get(), toClearshell](const auto &actor)
                       {

		auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

		if (*toClearshell) {
			renderer->RemoveActor(*toClearshell);
		}

		renderer->AddActor(actor);
		*toClearshell = actor;

		renderWindow->Render(); });
}

#include "../shell_space/CGALMeshShell.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"
#include "../shell_space/SphereShell.h"
#include "path_visualization.h"

ShellWrapper sphereShellWrapper(const std::shared_ptr<WorkspaceSphereShell> &shell)
{

    return ShellWrapper {
        [shell](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d>
        {
            return idealizedPathViaShell(*shell, from, to, 32);
        },
        "Spherical shell"
        };
}

ShellWrapper cuttingPlaneShellWrapper(const std::shared_ptr<CuttingPlaneConvexHullShell> &shell)
{

    return ShellWrapper {
        [shell](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d>
        {
            return idealizedPathViaShell(*shell, from, to, 32);
        },
        "Cutting plane convex hull shell"
        };
}

ShellWrapper cgalMeshShellWrapper(const std::shared_ptr<CGALMeshShell> &shell)
{

    return ShellWrapper {
        [shell](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d>
        {
            return idealizedPathViaShell(*shell, from, to, 32);
        },
        "CGAL mesh shell"
        };
}

rxcpp::observable<ShellWrapper> createShellWrapperObservable(
    const rxcpp::observable<std::shared_ptr<WorkspaceSphereShell>> &current_sphere_shell,
    const rxcpp::observable<std::shared_ptr<CuttingPlaneConvexHullShell>> &cutting_plane_chull_shell,
    const rxcpp::observable<std::shared_ptr<CGALMeshShell>> &cgal_mesh_shell,
    const rxcpp::observable<int> &current_shell_index)
{

    auto sphere_shell_wrapper = current_sphere_shell | Rx::map(sphereShellWrapper);

    auto cutting_plane_shell_wrapper = cutting_plane_chull_shell | Rx::map(cuttingPlaneShellWrapper);

    auto cgal_mesh_shell_wrapper = cgal_mesh_shell | Rx::map(cgalMeshShellWrapper);

    auto current_shell_wrapper = current_shell_index | Rx::combine_latest(sphere_shell_wrapper, cutting_plane_shell_wrapper, cgal_mesh_shell_wrapper) |
                                 Rx::map([](const auto &tuple) -> ShellWrapper
                                         {
									 auto [index, sphere_shell_wrapper, cutting_plane_shell_wrapper, cgal_mesh_shell_wrapper] = tuple;

									 std::cout << "Switching to shell wrapper " << index << std::endl;

									 switch (index) {
										 case 0:
											 return sphere_shell_wrapper;
										 case 1:
											 return cutting_plane_shell_wrapper;
										 case 2:
											 return cgal_mesh_shell_wrapper;
										 default:
											 throw std::runtime_error("Invalid shell index");
									 } });

    return current_shell_wrapper;
}