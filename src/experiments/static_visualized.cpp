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
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <rxqt.hpp>
#pragma GCC diagnostic pop

namespace Rx
{
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

#include "../utilities/vtk.h"
#include "../utilities/mesh_utils.h"
#include "../visualization/ActorsVisibilityWidget.h"
#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/SphereShell.h"
#include "../shell_space/CGALMeshShell.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"
#include "../utilities/enclosing_sphere.h"

#include "../visualization/numeric_combobox.h"
#include "../visualization/path_visualization.h"
#include "../visualization/rx_util.h"
#include "../visualization/camera_controls.h"
#include "../visualization/shell_visualization.h"
#include "../visualization/LabeledActors.h"
#include "../visualization/rx_vtk.h"
#include "../visualization/compute_prm.h"

#include "../TreeMeshes.h"

int main(int argc, char **argv) {


	SimpleVtkViewer viewer;


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

	current_scene_name.subscribe([=](const QString &sceneName)
								 { std::cout << "Signal current_scene_name: " << sceneName.toStdString() << std::endl; });

	auto current_tree_models = current_scene_name |
							   map_async_latest<QString, std::function<TreeMeshes(const QString &)>>([](const QString &sceneName) {
								   return loadTreeMeshes(sceneName.toStdString());
							   });

	current_tree_models.subscribe([=](const TreeMeshes &treeMeshes)
								  { std::cout << "Signal current_tree_models: " << treeMeshes.tree_name << std::endl; });

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

	auto current_labeled_actors = current_tree_models.map(treeMeshesToLabeledActors).publish().ref_count();

	updateRendererContents(current_labeled_actors, renderWindow, vtkWidget);

	auto actorsVisibilityWidget = new ActorsVisibilityWidget(renderWindow.Get());
	sidebarLayout->addWidget(actorsVisibilityWidget);

	current_labeled_actors.subscribe([actorsVisibilityWidget](const auto &actors_with_labels)
									 { actorsVisibilityWidget->setActorsWithLabels(actors_with_labels); });

	auto map_lambda = [](const TreeMeshes &treeMeshes) -> AppleTreePlanningScene
	{
		std::vector<Apple> apples;

		for (const auto &mesh : treeMeshes.fruit_meshes)
		{
			apples.push_back(appleFromMesh(mesh));
		}

		return AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(
										  std::move(treeMeshesToMoveitSceneMsg(treeMeshes))),
									  .apples = apples};
	};

	auto current_scene = current_tree_models | map_async_latest<TreeMeshes, std::function<AppleTreePlanningScene(const TreeMeshes &)>>(map_lambda);

	current_scene.subscribe([=](const auto &scene)
							{ std::cout << "Loaded scene " << scene.scene_msg->name << std::endl; });

	// sphere shell
	auto current_sphere_shell = current_scene |
								map_async_latest<AppleTreePlanningScene, std::function<std::shared_ptr<WorkspaceSphereShell>(
																			 const AppleTreePlanningScene &)>>([](const AppleTreePlanningScene &scene)
																											   {
									auto sphere_shell = paddedSphericalShellAroundLeaves(scene, 0.0);
									return sphere_shell; });

	// convex hull
	auto convex_hull = current_scene |
					   map_async_latest<AppleTreePlanningScene, std::function<shape_msgs::msg::Mesh(const AppleTreePlanningScene &)>>(
						   [](const AppleTreePlanningScene &scene)
						   {
							   return convexHull(utilities::extract_leaf_vertices(scene));
						   });

	// cutting plane convex hull shell
	auto cutting_plane_chull_shell = convex_hull | Rx::map([](const auto &convex_hull)
														   { return std::make_shared<CuttingPlaneConvexHullShell>(convex_hull, 0.0, 0.0); });

	// cgal mesh shell
	auto cgal_mesh_shell = convex_hull | Rx::map([](const auto &convex_hull)
												 { return std::make_shared<CGALMeshShell>(convex_hull, 0.0, 0.0); });

	auto comboBox = QSharedPointer<QComboBox>(new QComboBox(nullptr));
	comboBox->addItem("Sphere shell");
	comboBox->addItem("Cutting plane convex hull shell");
	comboBox->addItem("CGAL mesh shell");
	sidebarLayout->addWidget(comboBox.data());

	auto current_shell_index =
		rxqt::from_signal(comboBox.data(), static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged)) |
		Rx::start_with(comboBox->currentIndex());

	current_shell_index.subscribe([](const auto &index)
								  { std::cout << "Selected shell index " << index << std::endl; });

	auto sphere_actor = current_sphere_shell | Rx::map([](const auto &shell) -> vtkSmartPointer<vtkActor>
													   { return mkSphereShellActor(*shell); }) |
						Rx::publish() | Rx::ref_count();

	auto chull_actor = convex_hull | Rx::map([](const auto &shell) -> vtkSmartPointer<vtkActor>
											 {
		std::cout << "Created convex hull shell actor with " << shell.triangles.size() << " triangles" << std::endl;
		return createColoredMeshActor(shell, {0.9, 0.9, 0.9, 0.5}, true); }) |
					   Rx::publish() | Rx::ref_count();

	auto current_shell_actor = current_shell_index | Rx::combine_latest(sphere_actor, chull_actor) |
							   Rx::map([](const auto &tuple) -> vtkSmartPointer<vtkActor>
									   {

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
								   } });

	addReactiveActor(current_shell_actor, renderWindow);

	auto apple_id_box = createAppleIdComboBox(100); // scene.apples.size());
	sidebarLayout->addWidget(apple_id_box.get());

	VtkLineSegmentsVisualization path_viz(1.0, 0.0, 1.0);
	renderer->AddActor(path_viz.getActor());

	auto current_source_apple = rxqt::from_signal(apple_id_box.data(),
												  static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged));

	auto current_shell_wrapper = createShellWrapperObservable(
		current_sphere_shell,
		cutting_plane_chull_shell,
		cgal_mesh_shell,
		current_shell_index
	);

	current_scene | Rx::combine_latest(current_source_apple, current_shell_wrapper) |
		Rx::subscribe<std::tuple<AppleTreePlanningScene, int, ShellWrapper>>([&](const std::tuple<AppleTreePlanningScene, int, ShellWrapper> &tuple)
																			 {

		const auto &[scene, source_apple_index, shell] = tuple;

		std::cout << "Updating with apple " << source_apple_index << " updatePaths: " << shell.name << std::endl;

		// Update the path_viz with the new edges
		path_viz.updateLine(computeOneToAllIdealizedPathEdges(scene.apples, source_apple_index, shell.idealized_path)); 
	});

	// Add a button to the sidebar to build a PRM of the scene.
	auto build_prm_button = new QPushButton("Build PRM");
	sidebarLayout->addWidget(build_prm_button);

	// Get a signal when the button is clicked
	auto build_prm_clicked = rxqt::from_signal(build_prm_button, &QPushButton::clicked);

	// Subscribe and emit a println when the button is clicked
	build_prm_clicked.subscribe([&](const auto &t)
								{ std::cout << "Build PRM clicked" << t << std::endl; });

	// Asyncmap the button click to a future that will compute the PRM
	build_prm_clicked 
		| Rx::combine_latest(current_scene)
		| map_async_latest<std::pair<bool, AppleTreePlanningScene>, std::function<Roadmap(std::pair<bool, AppleTreePlanningScene>)>>([](auto tuple) {
			const auto &scene = std::get<1>(tuple);
			return computePRM(scene);
		}) 
		| Rx::subscribe<Roadmap>([](const Roadmap &prm) {std::cout << "Computed PRM with " << prm.size() << " nodes" << std::endl;});

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
