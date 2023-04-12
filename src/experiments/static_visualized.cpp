#include <utility>
#include <boost/asio.hpp>
#include <range/v3/all.hpp>

#include <CGAL/Delaunay_triangulation_3.h>

#include <vtkProperty.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCallbackCommand.h>
#include <vtkRendererCollection.h>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QSplitter>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <range/v3/view/drop.hpp>
#include <QFutureWatcher>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>

#include "../planning_scene_diff_message.h"
#include "../visualization/Viewer.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/shell_visualization.h"
#include "../visualization/camera_controls.h"
#include "../visualization/ObservableInt.h"
#include "../utilities/enclosing_sphere.h"

std::unique_ptr<QCheckBox> mkVisibilityCheckbox(std::vector<vtkSmartPointer<vtkActor>> actors,
												const std::string &label,
												vtkRenderWindow *renderer) {

	auto checkBox2 = std::make_unique<QCheckBox>(QString(label.c_str()));

	checkBox2->setChecked(actors.front()->GetVisibility());

	// Update visibility based on checkbox state
	QObject::connect(checkBox2.get(), &QCheckBox::stateChanged, [actors, renderer](int state) {

		std::cout << "state: " << state << std::endl;

		for (auto &actor: actors) {
			actor->SetVisibility(state == Qt::Checked);
			actor->Modified();
		}

		renderer->Render();

	});


	return checkBox2;

}

std::unique_ptr<QCheckBox>
mkVisibilityCheckbox(vtkSmartPointer<vtkActor> actor, const std::string &label, vtkRenderWindow *renderer) {

	std::vector actors = {actor};

	return mkVisibilityCheckbox(actors, label, renderer);
}

template<typename ShellPoint>
std::vector<Eigen::Vector3d> shellPathToPoints(const std::shared_ptr<ShellPath<ShellPoint>> &path,
											   const WorkspaceShell<ShellPoint> &shell,
											   int curve_steps) {

	std::vector<Eigen::Vector3d> points;

	if (auto curve = std::dynamic_pointer_cast<CurvePath<ShellPoint>>(path)) {

		for (int i = 0; i < curve_steps; i++) {
			auto t = i / (double) curve_steps;
			auto p = curve->at(t);
			auto surface_point = shell.surface_point(p);
			points.push_back(surface_point);
		}

	} else if (auto piecewise = std::dynamic_pointer_cast<PiecewiseLinearPath<ShellPoint>>(path)) {

		for (auto &p: piecewise->points) {
			points.push_back(shell.surface_point(p));
		}

	}

	return points;

}

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

// A Rust trait-object-like wrapper around the various WorkspaceShell types
struct ShellWrapper {
	std::function<std::vector<Eigen::Vector3d>(const Eigen::Vector3d &, const Eigen::Vector3d &)> idealized_path;
	const std::string &name;
};

QSharedPointer<QComboBox> createShellTypeComboBox(const std::vector<ShellWrapper> &shells, QWidget *parent = nullptr) {

	auto comboBox = QSharedPointer<QComboBox>(new QComboBox(parent));

	for (const auto &shell: shells) {
		comboBox->addItem(QString::fromStdString(shell.name));
	}

	return comboBox;
}

QSharedPointer<QComboBox> createAppleIdComboBox(const size_t num_apples, QWidget *parent = nullptr) {

	auto comboBox = QSharedPointer<QComboBox>(new QComboBox(parent));

	for (size_t i = 0; i < num_apples; i++) {
		comboBox->addItem(QString::number(i));
	}

	return comboBox;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeOneToAllIdealizedPathEdges(const std::vector<Apple> &apples, int apple_i, const ShellWrapper &wrapper) {
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

	for (size_t apple_j = 0; apple_j < apple_i; apple_j++) {

		// Get the range of points for the shell between the two apples
		std::vector<Eigen::Vector3d> shell_points;

		shell_points = wrapper.idealized_path(apples[apple_i].center, apples[apple_j].center);

		// Convert the range of points to a vector of pairs of points
		auto edges_for_path =
				ranges::views::zip(shell_points, shell_points | ranges::views::drop(1)) | ranges::to_vector;

		// Add the edges to the vector of edges
		edges.insert(edges.end(), edges_for_path.begin(), edges_for_path.end());
	}
	return edges;
}

struct ConvexHullShells {
	shape_msgs::msg::Mesh convex_hull;
	std::shared_ptr<CuttingPlaneConvexHullShell> cutting_plane_shell;
	std::shared_ptr<CGALMeshShell> cgal_mesh_shell;
};

ConvexHullShells compute_convex_hull_shells(const AppleTreePlanningScene &scene) {
	auto convex_hull = convexHull(utilities::extract_leaf_vertices(scene));
	auto chull_shell = std::make_shared<CuttingPlaneConvexHullShell>(convex_hull, 0.0, 0.0);
	auto cgal_shell = std::make_shared<CGALMeshShell>(convex_hull, 0.0, 0.0);

	return ConvexHullShells{.convex_hull = std::move(convex_hull), .cutting_plane_shell = chull_shell, .cgal_mesh_shell = cgal_shell};
}

int main(int argc, char **argv) {

	const QStringList sceneNames = {"appletree", "lemontree2", "orangetree4"};

	QApplication app(argc, argv);

	auto window = std::make_unique<QWidget>();

	auto layout = new QHBoxLayout();

	// Sidebar
	auto sidebarLayout = new QVBoxLayout();

	auto sceneComboBox = new QComboBox();
	sceneComboBox->addItems(sceneNames);
	sidebarLayout->addWidget(sceneComboBox);
	sceneComboBox->setCurrentIndex(0);

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

	QFutureWatcher<TreeMeshes> meshes_watcher;

	std::vector<vtkActor *> toClear;

	QObject::connect(sceneComboBox,
					 qOverload<int>(&QComboBox::currentIndexChanged),
					 [sceneNames, renderWindow = renderWindow.Get(), sidebarLayout, &meshes_watcher, &toClear](int index) {
						 assert(index >= 0 && index < sceneNames.size());

						 // Load the apple tree meshes.
						 QFuture<TreeMeshes> meshes_future = QtConcurrent::run([index, &sceneNames]() -> TreeMeshes {
							 return loadTreeMeshes(sceneNames[index].toStdString());
						 });

						 meshes_watcher.setFuture(meshes_future);

						 // Continue with the rest of the code when the meshes are loaded.
						 QObject::connect(&meshes_watcher,
										  &QFutureWatcher<TreeMeshes>::finished,
										  [renderWindow, meshes_future, sidebarLayout, &toClear]() {

											  auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();


											  for (auto actor: toClear) {
												  renderer->RemoveActor(actor);
											  }

											  // Get the meshes from the future.
											  const auto &meshes = meshes_future.result();

											  std::cout << "Meshes loaded" << std::endl;


											  auto trunk_actor = addColoredMeshActor(meshes.trunk_mesh,
																					 {0.5, 0.3, 0.1, 1.0},
																					 renderer);

											  toClear.push_back(trunk_actor);

											  auto leaves_actor = addColoredMeshActor(meshes.leaves_mesh,
																					  {0.1, 0.5, 0.1, 1.0},
																					  renderer);

											  toClear.push_back(leaves_actor);

											  auto ground_actor = addColoredMeshActor(createGroundPlane(10.0, 10.0),
																					  {0.5, 0.5, 0.1, 1.0},
																					  renderer);

											  toClear.push_back(ground_actor);

											  std::vector<vtkSmartPointer<vtkActor>> fruit_actors;
											  for (const auto &mesh: meshes.fruit_meshes) {
												  fruit_actors.push_back(addColoredMeshActor(mesh,
																							 {1.0, 0.0, 0.0, 1.0},
																							 renderer));
											  }

											  toClear.insert(toClear.end(), fruit_actors.begin(), fruit_actors.end());

											  sidebarLayout->addWidget(mkVisibilityCheckbox(trunk_actor,
																							"trunk",
																							renderWindow).release());
											  sidebarLayout->addWidget(mkVisibilityCheckbox(leaves_actor,
																							"leaves",
																							renderWindow).release());
											  sidebarLayout->addWidget(mkVisibilityCheckbox(ground_actor,
																							"ground",
																							renderWindow).release());
											  sidebarLayout->addWidget(mkVisibilityCheckbox(fruit_actors,
																							"apples",
																							renderWindow).release());

											  renderWindow->Render();

										  });

					 });


	//	QFuture<AppleTreePlanningScene> scene_future = chainQFuture(meshes_future, [](const TreeMeshes &meshes) {
	//
	//		auto apples = meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector;
	//
	//		return AppleTreePlanningScene{
	//				.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(treeMeshesToMoveitSceneMsg(meshes))),
	//				.apples = apples
	//		};
	//
	//	});
	//
	//	QFuture<std::shared_ptr<WorkspaceSphereShell>> sphere_shell_future = chainQFuture(scene_future, [](const AppleTreePlanningScene &scene) {
	//		return paddedSphericalShellAroundLeaves(scene, 0.0);
	//	});
	//
	//	QFuture<ConvexHullShells> convex_hull_shells_future = chainQFuture(scene_future, compute_convex_hull_shells);

	//
	//	QFutureWatcher<AppleTreePlanningScene> scene_watcher;
	//	scene_watcher.setFuture(scene_future);
	//
	//	QObject::connect(&scene_watcher, &QFutureWatcher<AppleTreePlanningScene>::finished, [&]() {
	//
	//		const auto &scene = scene_future.result();
	//
	////		auto robot = loadRobotModel();
	//
	////		auto ss = omplStateSpaceForDrone(robot);
	////		auto si = loadSpaceInformation(ss, scene);
	//
	//		auto sphereshell = paddedSphericalShellAroundLeaves(scene, 0.0);
	//
	////
	//		{
	//			vtkNew<vtkActor> sphereActor = mkSphereShellActor(*sphereshell);
	//			renderer->AddActor(sphereActor);
	//			sidebarLayout->addWidget(mkVisibilityCheckbox(sphereActor, "spherical shell", renderWindow).release());
	//		}
	//
	//		{
	//			auto chull_actor = addColoredMeshActor(convex_hull, {0.8, 0.8, 0.8, 0.5}, renderer, false);
	//			sidebarLayout->addWidget(mkVisibilityCheckbox(chull_actor, "convex hull", renderWindow).release());
	//		}
	//
	//		std::vector<ShellWrapper> shells = {ShellWrapper{
	//				[&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
	//					return idealizedPathViaShell(*sphereshell, from, to, 32);
	//				}, "Spherical shell"}, ShellWrapper{
	//				[&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
	//					return idealizedPathViaShell(*chull_shell, from, to, 32);
	//				}, "Cutting plane convex hull"}, ShellWrapper{
	//				[&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
	//					return idealizedPathViaShell(*cgal_shell, from, to, 32);
	//				}, "CGAL convex hull"}};
	//
	//		auto shell_type_box = createShellTypeComboBox(shells);
	//		sidebarLayout->addWidget(shell_type_box.get());
	//
	//		auto apple_id_box = createAppleIdComboBox(scene.apples.size());
	//		sidebarLayout->addWidget(apple_id_box.get());
	//
	//		VtkLineSegmentsVisualization path_viz(1.0, 0.0, 1.0);
	//
	//		renderer->AddActor(path_viz.getActor());
	//
	//		auto updatePathsMulti = [&]() {
	//			int apple_i = apple_id_box->currentIndex();
	//			int shell_i = shell_type_box->currentIndex();
	//
	//			const auto wrapper = shells[shell_i];
	//
	//			std::cout << "Updating with apple " << apple_i << " updatePathsMulti: " << apple_i << std::endl;
	//
	//			// Update the path_viz with the new edges
	//			path_viz.updateLine(computeOneToAllIdealizedPathEdges(scene.apples, apple_i, wrapper));
	//		};
	//
	//		// Call the function when either changes.
	//		QObject::connect(apple_id_box.get(), QOverload<int>::of(&QComboBox::currentIndexChanged), updatePathsMulti);
	//		QObject::connect(shell_type_box.get(),
	//						 QOverload<int>::of(&QComboBox::currentIndexChanged),
	//						 updatePathsMulti);
	//
	//	});

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
