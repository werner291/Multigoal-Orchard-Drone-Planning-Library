#include <range/v3/all.hpp>
#include <boost/asio.hpp>

#include "../planning_scene_diff_message.h"
#include "../vtk/Viewer.h"

#include "../vtk/SimpleVtkViewer.h"

#include <vtkProperty.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QApplication>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QSplitter>
#include <utility>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkSphereSource.h>
#include <vtkCallbackCommand.h>
#include <QLabel>
#include <QComboBox>

vtkNew<vtkActor> mkSphereShellActor(const std::shared_ptr<WorkspaceSphereShell> &sphereshell);

void enforceCameraUp(vtkNew<vtkRenderer> &renderer, const QVTKOpenGLNativeWidget *vtkWidget);

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

vtkSmartPointer<vtkActor> createAndAddActor(const shape_msgs::msg::Mesh &mesh,
											const std::array<double, 3> &color_rgb,
											const std::string &name,
											vtkSmartPointer<vtkRenderer> renderer,
											QBoxLayout *sidebarLayout,
											vtkSmartPointer<vtkRenderWindow> renderWindow) {

	auto actor = createActorFromMesh(mesh);
	actor->GetProperty()->SetColor(color_rgb[0], color_rgb[1], color_rgb[2]);
	renderer->AddActor(actor);
	sidebarLayout->addWidget(mkVisibilityCheckbox(actor, name, renderWindow).release());

	return actor;
}

vtkSmartPointer<vtkActor> createAndAddActor(const shape_msgs::msg::Mesh &mesh,
											double r,
											double g,
											double b,
											const std::string &name,
											vtkSmartPointer<vtkRenderer> renderer,
											QBoxLayout *sidebarLayout,
											vtkSmartPointer<vtkRenderWindow> renderWindow) {

	auto actor = createActorFromMesh(mesh);
	actor->GetProperty()->SetColor(r, g, b);
	renderer->AddActor(actor);
	sidebarLayout->addWidget(mkVisibilityCheckbox(actor, name, renderWindow).release());

	return actor;
}

std::vector<vtkSmartPointer<vtkActor>>
createAndAddActorsWithSharedCheckbox(const std::vector<shape_msgs::msg::Mesh> &meshes,
									 double r,
									 double g,
									 double b,
									 const std::string &name,
									 vtkSmartPointer<vtkRenderer> renderer,
									 QBoxLayout *sidebarLayout,
									 vtkSmartPointer<vtkRenderWindow> renderWindow) {

	std::vector<vtkSmartPointer<vtkActor>> actors;

	for (const auto &mesh: meshes) {
		auto actor = createActorFromMesh(mesh);
		actor->GetProperty()->SetColor(r, g, b);
		renderer->AddActor(actor);
		actors.emplace_back(actor);
	}

	sidebarLayout->addWidget(mkVisibilityCheckbox(actors, name, renderWindow).release());
	return actors;
}

// A Rust trait-object-like wrapper around the various WorkspaceShell types
struct ShellWrapper {
	std::function<std::vector<Eigen::Vector3d>(const Eigen::Vector3d &, const Eigen::Vector3d &)> idealized_path;
	const std::string &name;
};

int main(int argc, char **argv) {

	QApplication app(argc, argv);

	auto window = std::make_unique<QWidget>();

	auto layout = new QHBoxLayout();

	// Sidebar
	auto sidebarLayout = new QVBoxLayout();

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

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");
	auto apples = meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector;

	auto trunk_actor = createAndAddActor(meshes.trunk_mesh,
										 0.5,
										 0.3,
										 0.1,
										 "trunk",
										 renderer,
										 sidebarLayout,
										 renderWindow);
	auto leaves_actor = createAndAddActor(meshes.leaves_mesh,
										  0.1,
										  0.5,
										  0.1,
										  "leaves",
										  renderer,
										  sidebarLayout,
										  renderWindow);
	auto ground_plane_actor = createAndAddActor(createGroundPlane(10.0, 10.0),
												0.5,
												0.5,
												0.1,
												"ground",
												renderer,
												sidebarLayout,
												renderWindow);
	auto apple_actors = createAndAddActorsWithSharedCheckbox(meshes.fruit_meshes,
															 0.8,
															 0.2,
															 0.2,
															 "apples",
															 renderer,
															 sidebarLayout,
															 renderWindow);

	auto scene = AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(
			treeMeshesToMoveitSceneMsg(meshes))), .apples = apples};

	auto robot = loadRobotModel();

	auto ss = omplStateSpaceForDrone(robot);
	auto si = loadSpaceInformation(ss, scene);

	auto sphereshell = paddedSphericalShellAroundLeaves(scene, 0.0);

	{
		vtkNew<vtkActor> sphereActor = mkSphereShellActor(sphereshell);
		renderer->AddActor(sphereActor);
		sidebarLayout->addWidget(mkVisibilityCheckbox(sphereActor, "spherical shell", renderWindow).release());
	}

	auto convex_hull = convexHull(meshes.leaves_mesh.vertices);
	auto chull_shell = std::make_shared<CuttingPlaneConvexHullShell>(convex_hull, 0.0, 0.0);
	auto cgal_shell = std::make_shared<CGALMeshShell>(convex_hull, 0.0, 0.0);

	{
		// Convex hull
		auto chull_actor = createActorFromMesh(convex_hull);
		chull_actor->GetProperty()->SetColor(0.8, 0.8, 0.8);
		chull_actor->GetProperty()->SetOpacity(0.5);

		renderer->AddActor(chull_actor);
		sidebarLayout->addWidget(mkVisibilityCheckbox(chull_actor, "convex hull", renderWindow).release());
	}

	std::vector<ShellWrapper> shells = {
			ShellWrapper{[&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
				return idealizedPathViaShell(*sphereshell, from, to, 32);
			}, "Spherical shell"},
			ShellWrapper{[&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
				return idealizedPathViaShell(*chull_shell, from, to, 32);
			}, "Cutting plane convex hull"},
			ShellWrapper{[&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) -> std::vector<Eigen::Vector3d> {
				return idealizedPathViaShell(*cgal_shell, from, to, 32);
			}, "CGAL convex hull"}};

	auto shell_type_box = new QComboBox();
	for (const auto &shell: shells) {
		shell_type_box->addItem(QString::fromStdString(shell.name));
	}

	// Add a slider to select an apple ID
	auto appleIDComboBox = new QComboBox();
	for (int i = 0; i < apples.size(); i++) {
		appleIDComboBox->addItem(QString("Apple ID: %1").arg(i));
	}

	// Add a label to display the selected apple ID
	auto appleIDLabel = new QLabel();
	appleIDLabel->setText("Apple ID: 0");

	VtkLineSegmentsVisualization path_viz(1.0, 0.0, 1.0);

	renderer->AddActor(path_viz.getActor());

	auto updatePathsMulti = [&]() {

		int apple_i = appleIDComboBox->currentIndex();

		std::cout << "Updating with apple " << apple_i << std::endl;

		std::cout << "updatePathsMulti: " << apple_i << std::endl;

		// Update the QLabel
		appleIDLabel->setText(QString("Apple ID").arg(apple_i));

		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

		for (size_t apple_j = 0; apple_j < apple_i; apple_j++) {

			// Get the range of points for the shell between the two apples
			std::vector<Eigen::Vector3d> shell_points;

			shell_points = shells[shell_type_box->currentIndex()].idealized_path(apples[apple_i].center,
																				 apples[apple_j].center);

			// Convert the range of points to a vector of pairs of points
			auto edges_for_path =
					ranges::views::zip(shell_points, shell_points | ranges::views::drop(1)) | ranges::to_vector;

			// Add the edges to the vector of edges
			edges.insert(edges.end(), edges_for_path.begin(), edges_for_path.end());
		}

		// Update the path_viz with the new edges
		path_viz.updateLine(edges);
	};

	QObject::connect(appleIDComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), updatePathsMulti);
	QObject::connect(shell_type_box, QOverload<int>::of(&QComboBox::currentIndexChanged), updatePathsMulti);

	sidebarLayout->addWidget(shell_type_box);

	sidebarLayout->addWidget(appleIDLabel);
	sidebarLayout->addWidget(appleIDComboBox);
	enforceCameraUp(renderer, vtkWidget);

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

void enforceCameraUp(vtkNew<vtkRenderer> &renderer,
					 const QVTKOpenGLNativeWidget *vtkWidget) {// After setting the render window for the QVTKOpenGLNativeWidget
	vtkRenderWindowInteractor *interactor = vtkWidget->interactor();

	// Create a vtkCallbackCommand for the observer
	vtkNew<vtkCallbackCommand> resetViewUpCallback;

	// Set the callback function using a lambda
	resetViewUpCallback->SetCallback([](vtkObject *caller, unsigned long eventId, void *clientData, void *callData) {
		auto *interactor = dynamic_cast<vtkRenderWindowInteractor *>(caller);
		auto *renderer = static_cast<vtkRenderer *>(clientData);
		vtkCamera *camera = renderer->GetActiveCamera();
		camera->SetViewUp(0.0, 0.0, 1.0);
	});

	renderer->GetActiveCamera()->SetPosition(10.0, 0.0, 3.0);
	renderer->GetActiveCamera()->SetFocalPoint(0.0, 0.0, 2.0);
	renderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);

	// Set the clientData (the renderer in this case)
	resetViewUpCallback->SetClientData(renderer.GetPointer());

	// Add an observer for the EndInteractionEvent
	unsigned long observerId = interactor->AddObserver(vtkCommand::InteractionEvent, resetViewUpCallback);
}

vtkNew<vtkActor> mkSphereShellActor(const std::shared_ptr<WorkspaceSphereShell> &sphereshell) {// Spherical actor
	vtkNew<vtkSphereSource> sphereSource;
	sphereSource->SetCenter(sphereshell->getCenter().x(), sphereshell->getCenter().y(), sphereshell->getCenter().z());
	sphereSource->SetRadius(sphereshell->getRadius());
	sphereSource->SetThetaResolution(32);
	sphereSource->SetPhiResolution(16);

	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);

	sphereActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
	sphereActor->GetProperty()->SetOpacity(0.5);

	return sphereActor;
}




