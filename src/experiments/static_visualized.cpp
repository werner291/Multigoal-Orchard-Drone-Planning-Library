#include <utility>

#include <QApplication>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QComboBox>
#include <QSplitter>
#include <QVTKOpenGLNativeWidget.h>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"

#include <rxqt.hpp>

#pragma GCC diagnostic pop

namespace Rx {
	using namespace rxcpp;
	using namespace rxcpp::sources;
	using namespace rxcpp::operators;
	using namespace rxcpp::util;
}

#include <boost/asio.hpp>

#include <range/v3/all.hpp>
#include <range/v3/view/drop.hpp>

#include <CGAL/Delaunay_triangulation_3.h>

#include <vtkProperty.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCallbackCommand.h>
#include <vtkRendererCollection.h>

#include "../planning_scene_diff_message.h"
#include "../utilities/enclosing_sphere.h"
#include "../visualization/ActorsVisibilityWidget.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/Viewer.h"
#include "../visualization/camera_controls.h"
#include "../visualization/rx_util.h"
#include "../visualization/shell_visualization.h"
#include "../shell_space/path_util.h"

template<typename ShellPoint>
std::vector<Eigen::Vector3d> idealizedPathViaShell(const WorkspaceShell<ShellPoint> &shell,
												   const Eigen::Vector3d &start,
												   const Eigen::Vector3d &goal,
												   int curve_steps) {

	auto shell_pt1 = shell.nearest_point_on_shell(start);
	auto shell_pt2 = shell.nearest_point_on_shell(goal);

	auto path = shell.path_from_to(shell_pt1, shell_pt2);

	auto path_points = shellPathToPoints(path, shell, curve_steps);

	//	// Add start and goal

	path_points.insert(path_points.begin(), start);
	path_points.push_back(goal);

	return path_points;

}

QSharedPointer<QComboBox> createAppleIdComboBox(const size_t num_apples, QWidget *parent = nullptr) {

	auto comboBox = QSharedPointer<QComboBox>(new QComboBox(parent));

	for (size_t i = 0; i < num_apples; i++) {
		comboBox->addItem(QString::number(i));
	}

	return comboBox;
}

int main(int argc, char **argv) {

	const QStringList sceneNames = {"appletree", "lemontree2", "orangetree4"};

	QApplication app(argc, argv);
	rxqt::run_loop rxqt_run_loop;

	auto window = std::make_unique<QWidget>();

	auto layout = new QHBoxLayout();

	// Sidebar
	auto sidebarLayout = new QVBoxLayout();

	auto sceneComboBox = new QComboBox();
	sceneComboBox->addItems(sceneNames);
	sidebarLayout->addWidget(sceneComboBox);
	sceneComboBox->setCurrentIndex(0);

	auto current_scene_name =
			rxqt::from_signal(sceneComboBox, &QComboBox::currentTextChanged) | Rx::start_with(sceneNames[0]);

	current_scene_name.subscribe([=](const QString &sceneName) {
		std::cout << "Signal current_scene_name: " << sceneName.toStdString() << std::endl;
	});

	auto current_tree_models = current_scene_name |
							   map_async_latest<QString, std::function<TreeMeshes(const QString &)>>([](const QString &sceneName) {
								   return loadTreeMeshes(sceneName.toStdString());
							   });

	current_tree_models.subscribe([=](const TreeMeshes &treeMeshes) {
		std::cout << "Signal current_tree_models: " << treeMeshes.tree_name << std::endl;
	});

	// VTK render window
	vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
	vtkNew<vtkRenderer> renderer;

	// nice sky color
	renderer->SetBackground(0.5, 0.5, 1.0);

	renderWindow->AddRenderer(renderer);

	// QWidget wrapper for the VTK render window
	auto vtkWidget = new QVTKOpenGLNativeWidget();
	vtkWidget->setRenderWindow(renderWindow);
	vtkWidget->setMinimumSize(800, 600);
	enforceCameraUp(renderer, vtkWidget->interactor());

	auto current_labeled_actors = current_tree_models.map([](const TreeMeshes &treeMeshes) {
		return {{{createColoredMeshActor(meshes.trunk_mesh,{0.5, 0.3, 0.1, 1.0})},                                                                                    "trunk"},
				{{createColoredMeshActor(meshes.leaves_mesh,{0.1, 0.5,0.1,1.0})}, "leaves"},
				{{createColoredMeshActor(createGroundPlane(10.0,10.0),{0.5, 0.5, 0.1, 1.0})}, "ground"},
				{{createColoredMeshActors(meshes.fruit_meshes,{1.0, 0.0, 0.0, 1.0})}, "apples"}};
	}).publish().ref_count();

	current_labeled_actors.subscribe([=](const auto &actors_with_labels) {
		std::cout << "Signal current_labeled_actors: " << actors_with_labels.size() << std::endl;
	});

	auto toClear = std::make_shared<std::vector<vtkActor * >>();
	current_labeled_actors.subscribe([renderWindow = renderWindow.Get(), &toClear, vtkWidget](const auto &actors_with_labels) {
		auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

		for (auto actor: *toClear) {
			renderer->RemoveActor(actor);
		}

		for (const auto &[actors, label]: actors_with_labels) {
			for (const auto &actor: actors) {
				renderer->AddActor(actor);
				toClear->push_back(actor);
			}
		}

		renderer->ResetCamera();
		vtkWidget->update();
		renderWindow->Render();
	});

	auto actorsVisibilityWidget = new ActorsVisibilityWidget(renderWindow.Get());
	sidebarLayout->addWidget(actorsVisibilityWidget);

	current_labeled_actors.subscribe([actorsVisibilityWidget](const auto &actors_with_labels) {
		actorsVisibilityWidget->setActorsWithLabels(actors_with_labels);
	});

	auto current_scene = current_tree_models |
						 map_async_latest<TreeMeshes, std::function<AppleTreePlanningScene(const TreeMeshes &)>>([](
								 const TreeMeshes &treeMeshes) {
							 auto apples = treeMeshes.fruit_meshes | ranges::views::transform(appleFromMesh) |
										   ranges::to_vector;

							 return AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(
									 std::move(treeMeshesToMoveitSceneMsg(treeMeshes))), .apples = apples};
						 });

	current_scene.subscribe([=](const auto &scene) {
		std::cout << "Loaded scene " << scene.scene_msg->name << std::endl;
	});

	// sphere shell
	auto current_sphere_shell = current_scene |
								map_async_latest<AppleTreePlanningScene, std::function<std::shared_ptr<WorkspaceSphereShell>(
										const AppleTreePlanningScene &)>>([](const AppleTreePlanningScene &scene) {
									auto sphere_shell = paddedSphericalShellAroundLeaves(scene, 0.0);
									return sphere_shell;
								});

	// convex hull
	auto convex_hull = current_scene |
					   map_async_latest<AppleTreePlanningScene, std::function<shape_msgs::msg::Mesh(const AppleTreePlanningScene &)>>(
							   [](const AppleTreePlanningScene &scene) {
								   return convexHull(utilities::extract_leaf_vertices(scene));
							   });

	convex_hull.subscribe([](const auto &convex_hull) {
		std::cout << "Created convex hull" << std::endl;
	});

	// cutting plane convex hull shell
	auto cutting_plane_chull_shell = convex_hull | Rx::map([](const auto &convex_hull) {
		return std::make_shared<CuttingPlaneConvexHullShell>(convex_hull, 0.0, 0.0);
	});

	cutting_plane_chull_shell.subscribe([](const auto &shell) {
		std::cout << "Created cutting plane convex hull shell" << std::endl;
	});

	// cgal mesh shell
	auto cgal_mesh_shell = convex_hull | Rx::map([](const auto &convex_hull) {
		return std::make_shared<CGALMeshShell>(convex_hull, 0.0, 0.0);
	});

	cgal_mesh_shell.subscribe([](const auto &shell) {
		std::cout << "Created CGAL mesh shell" << std::endl;
	});

	auto comboBox = QSharedPointer<QComboBox>(new QComboBox(nullptr));
	comboBox->addItem("Sphere shell");
	comboBox->addItem("Cutting plane convex hull shell");
	comboBox->addItem("CGAL mesh shell");
	sidebarLayout->addWidget(comboBox.data());

	auto current_shell_index =
			rxqt::from_signal(comboBox.data(), static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged)) |
			Rx::start_with(comboBox->currentIndex());

	current_shell_index.subscribe([](const auto &index) {
		std::cout << "Selected shell index " << index << std::endl;
	});

	auto sphere_actor = current_sphere_shell | Rx::map([](const auto &shell) -> vtkSmartPointer<vtkActor> {
		return mkSphereShellActor(*shell);
	}) | Rx::publish() | Rx::ref_count();

	sphere_actor.subscribe([](const auto &actor) {
		std::cout << "Created sphere shell actor" << std::endl;
	});

	auto chull_actor = convex_hull | Rx::map([](const auto &shell) -> vtkSmartPointer<vtkActor> {
		std::cout << "Created convex hull shell actor with " << shell.triangles.size() << " triangles" << std::endl;
		return createColoredMeshActor(shell, {0.9, 0.9, 0.9, 0.5}, true);
	}) | Rx::publish() | Rx::ref_count();

	chull_actor.subscribe([](const auto &actor) {
		std::cout << "Created convex hull shell actor" << std::endl;
	});

	auto current_shell_actor = current_shell_index | Rx::combine_latest(sphere_actor, chull_actor) |
							   Rx::map([](const auto &tuple) -> vtkSmartPointer<vtkActor> {

								   int index = std::get<0>(tuple);
								   vtkSmartPointer<vtkActor> sphere = std::get<1>(tuple);
								   vtkSmartPointer<vtkActor> chull = std::get<2>(tuple);

								   std::cout << "Switching shell actor to index " << index << std::endl;

								   switch (index) {
									   case 0:
										   return sphere;
									   case 1:
									   case 2:
										   return chull;
									   default:
										   throw std::runtime_error("Invalid shell index");
								   }
							   });

	current_shell_actor.subscribe([](const auto &actor) {
		std::cout << "Created shell actor" << std::endl;
	});

	// Put that actor into the renderer as an actor, keeping track of the old actor so we can remove it.
	auto toClearshell = std::make_shared<vtkSmartPointer<vtkActor >>();

	current_shell_actor.subscribe([renderWindow = renderWindow.Get(), &toClearshell](const auto &actor) {

		std::cout << "Added shell actor to scene " << (long) actor.GetPointer() << std::endl;

		auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

		if (*toClearshell) {
			renderer->RemoveActor(*toClearshell);
		}

		renderer->AddActor(actor);
		*toClearshell = actor;

		renderWindow->Render();
	});

	auto apple_id_box = createAppleIdComboBox(100);//scene.apples.size());
	sidebarLayout->addWidget(apple_id_box.get());

	VtkLineSegmentsVisualization path_viz(1.0, 0.0, 1.0);
	renderer->AddActor(path_viz.getActor());

	auto current_source_apple = rxqt::from_signal(apple_id_box.data(),
												  static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged));

	// Output the current source apple
	current_source_apple.subscribe([=](const auto &apple_id) {
		std::cout << "Current source apple: " << apple_id << std::endl;
	});

	QObject::connect(apple_id_box.get(),
					 static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
					 [](int index) {
						 std::cout << "Qt signal: Selected apple index " << index << std::endl;
					 });

	auto sphere_shell_wrapper = current_sphere_shell | Rx::map([](const auto &shell) -> ShellWrapper {
		return ShellWrapper{
				[shell](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
					return idealizedPathViaShell(*shell, from, to, 32);
				}, "Spherical shell"};
	});

	sphere_shell_wrapper.subscribe([=](const auto &shell_wrapper) {
		std::cout << "Created sphere shell wrapper" << std::endl;
	});

	auto cutting_plane_shell_wrapper = cutting_plane_chull_shell | Rx::map([](const auto &shell) -> ShellWrapper {
		return ShellWrapper{
				[shell](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
					return idealizedPathViaShell(*shell, from, to, 32);
				}, "Cutting plane convex hull shell"};
	});

	cutting_plane_shell_wrapper.subscribe([=](const auto &shell_wrapper) {
		std::cout << "Created cutting plane convex hull shell wrapper" << std::endl;
	});

	auto cgal_mesh_shell_wrapper = cgal_mesh_shell | Rx::map([](const auto &shell) -> ShellWrapper {
		return ShellWrapper{
				[shell](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
					return idealizedPathViaShell(*shell, from, to, 32);
				}, "CGAL mesh shell"};
	});

	cgal_mesh_shell_wrapper.subscribe([=](const auto &shell_wrapper) {
		std::cout << "Created CGAL mesh shell wrapper" << std::endl;
	});

	auto current_shell_wrapper = current_shell_index | Rx::combine_latest(sphere_shell_wrapper,
																		  cutting_plane_shell_wrapper,
																		  cgal_mesh_shell_wrapper) |
								 Rx::map([](const auto &tuple) -> ShellWrapper {
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
									 }
								 });

	current_shell_wrapper.subscribe([=](const auto &shell_wrapper) {
		std::cout << "Switched to shell wrapper" << shell_wrapper.name << std::endl;
	});


	current_scene | Rx::combine_latest(current_source_apple, current_shell_wrapper) |
	Rx::subscribe<std::tuple<AppleTreePlanningScene, int, ShellWrapper>>([&](const std::tuple<AppleTreePlanningScene, int, ShellWrapper> &tuple) {

		const auto &[scene, source_apple_index, shell] = tuple;

		std::cout << "Updating with apple " << source_apple_index << " updatePaths: " << shell.name << std::endl;

		// Update the path_viz with the new edges
		path_viz.updateLine(computeOneToAllIdealizedPathEdges(scene.apples, source_apple_index, shell.idealized_path));
	});

	////	//
	////	//		auto updatePathsMulti = [&]() {
	////	//			int apple_i = apple_id_box->currentIndex();
	////	//			int shell_i = shell_type_box->currentIndex();
	////	//
	////	//			const auto wrapper = shells[shell_i];
	////	//
	////	//			std::cout << "Updating with apple " << apple_i << " updatePathsMulti: " << apple_i << std::endl;
	////	//
	////	//			// Update the path_viz with the new edges
	////	//			path_viz.updateLine(computeOneToAllIdealizedPathEdges(scene.apples, apple_i, wrapper));
	////	//		};
	////	//
	////	//		// Call the function when either changes.
	////	//		QObject::connect(apple_id_box.get(), QOverload<int>::of(&QComboBox::currentIndexChanged), updatePathsMulti);
	////	//		QObject::connect(shell_type_box.get(),
	////	//						 QOverload<int>::of(&QComboBox::currentIndexChanged),
	////	//						 updatePathsMulti);
	////	//
	////	//	});
	////
	sidebarLayout->addStretch(1);

	// Splitter to separate the sidebar and the VTK render window
	auto splitter = new QSplitter();
	splitter->setOrientation(Qt::Horizontal);

	auto sidebarWidget = new QWidget();
	sidebarWidget->setLayout(sidebarLayout);
	splitter->addWidget(sidebarWidget);
	splitter->addWidget(vtkWidget);

	layout->addWidget(splitter);

	window->setLayout(layout);

	window->showMaximized();

	return app.exec();
}
