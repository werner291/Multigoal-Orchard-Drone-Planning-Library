
#ifndef NEW_PLANNERS_VTK_H
#define NEW_PLANNERS_VTK_H

#include <vtkPolyDataMapper.h>
#include <vtkLight.h>
#include <vtkCamera.h>
#include <vtkActorCollection.h>
#include <vtkCommand.h>
#include <vtkDepthImageToPointCloud.h>
#include <vtkRenderWindowInteractor.h>

#include <moveit/robot_model/link_model.h>

#include <shape_msgs/msg/mesh.hpp>
#include "../ScannablePointsIndex.h"
#include "../TreeMeshes.h"

static const int SENSOR_RESOLUTION = 200;

/**
 * Returns a vtkPolyDataMapper for the collision geometry of the given robot LinkModel.
 *
 * @param lm 		The LinkModel to get the collision geometry for.
 * @return 			A vtkPolyDataMapper for the collision geometry of the given robot LinkModel.
 */
vtkNew<vtkPolyDataMapper> polyDataForLink(const moveit::core::LinkModel *lm);

/**
 * Build a pure white ambient light, to bring out the ambient color of the objects without modification (and preserve the color encoding).
 *
 * @return 			A pure white ambient light.
 */
vtkNew<vtkLight> mkWhiteAmbientLight();

/**
 * Set the given camera to the given transform.
 *
 * @param tf 			The transform to set the camera to.
 * @param pCamera 		The camera to set the transform for.
 */
void setCameraFromEigen(Eigen::Isometry3d &tf, vtkCamera *pCamera);

/**
 * Add the given actor collection to the given renderer.
 *
 * @param orchard_actors 		The actor collection to add to the renderer.
 * @param sensorRenderer 		The renderer to add the actor collection to.
 */
void addActorCollectionToRenderer(vtkNew<vtkActorCollection> &orchard_actors, vtkNew<vtkRenderer> &sensorRenderer);

/**
 * Convert a ROS shape_msgs::msg::Mesh to a vtkPolyData.
 * @param mesh 		The ROS mesh to convert.
 * @return 			The converted vtkPolyData.
 */
vtkNew<vtkPolyData> rosMeshToVtkPolyData(const shape_msgs::msg::Mesh &mesh);

/**
 * Create a vtkActor with the given ROS mesh message.
 *
 * @param mesh 		The ROS mesh message to create the actor for.
 * @return 			The created vtkActor.
 */
vtkNew<vtkActor> createActorFromMesh(const shape_msgs::msg::Mesh &mesh);

/**
 * Extract a point cloud (source) from the given renderer. It will update when the renderer outputs a new frame.
 *
 * @param sensorRenderer 		The renderer to extract the point cloud from.
 * @return 						The extracted point cloud.
 */
vtkNew<vtkDepthImageToPointCloud> extractPointCloudFromRenderer(vtkNew<vtkRenderer> &sensorRenderer);

/**
 * A vtkTimerCommand that calls the given callback when the timer fires, to allow the use of lambdas as callbacks in vtk.
 *
 * Do not forget to call setCallback() to set the callback to call when the timer fires.
 */
class vtkFunctionalCallback : public vtkCommand
{
	/// The callback to call when the timer fires.
	std::function<void()> callback;
	unsigned long event_id{};
public:
	void setEventId(unsigned long eventId);

private:

	/// Constructor is private, use the static method New (or vtkNew) to create a new instance.
	vtkFunctionalCallback() = default;
public:

	/// Create a new vtkFunctionalCallback instance (usually through vtkNew).
	static vtkFunctionalCallback* New();

	/// Execute the callback. This simply calls the callback that was set earlier.
	virtual void Execute(vtkObject* caller, unsigned long eventId,
						 void* vtkNotUsed(callData));

	/// Set the callback to call when the timer fires.
	void setCallback(const std::function<void()> &cb) {
		vtkFunctionalCallback::callback = cb;
	}
};

/**
 * Build a vtkRenderer with a simple blue background and a diffuse light.
 *
 * @return The created vtkRenderer.
 */
vtkNew<vtkRenderer> buildViewerRenderer();

/**
 * Build a viewer window to go with the vtkRenderer from buildViewerRenderer.
 *
 * @param viewerRenderer 		The renderer to build the viewer window for.
 * @return 						The created viewer window.
 */
vtkNew<vtkRenderWindow> buildViewerWindow(vtkNew<vtkRenderer> &viewerRenderer);

/**
 * Build a simple window interactor that sets up a 33ms timer as well.
 * @param visualizerWindow 		The window to build the interactor for.
 * @return 						The created interactor.
 */
vtkNew<vtkRenderWindowInteractor> buildVisualizerWindowInteractor(vtkNew<vtkRenderWindow> &visualizerWindow);

/**
 * Construct a vtkActor for the given vtkPolyData, assuming that the latter represents a point cloud.
 *
 * @param pointCloudPolyData 		The vtkPolyData to construct the actor for.
 * @return 							The constructed vtkActor.
 */
vtkNew<vtkActor> constructSimplePolyDataPointCloudActor(const vtkNew<vtkPolyData> &pointCloudPolyData);

/**
 * Build vtkPolyData from the given vector of ScanTargetPoint, in order to visualize them.
 *
 * The resulting poly data will have one point and one vertex for every ScanTargetPoint,
 * colored dark blue, with matching indices.
 *
 * @param scan_targets 		The ScanTargetPoints to build the poly data for.
 * @return 					The built vtkPolyData.
 */
vtkNew<vtkPolyData> mkVtkPolyDataFromScannablePoints(const std::vector<ScanTargetPoint> &scan_targets);

/**
 * Build a vtkRenderer to simulate a sensor, with a black background and a white ambient light,
 * which preserves 1-to-1 the ambient color of all objects in the scene.
 * 
 * @return 		The created vtkRenderer.
 */
vtkNew<vtkRenderer> buildSensorRenderer();

/**
 * Build a render window for the sensor render.
 * 
 * @param sensorRenderer 		The sensor renderer to build the render window for.
 * @return 						The created render window.
 */
vtkNew<vtkRenderWindow> buildSensorRenderWindow(vtkNew<vtkRenderer> &sensorRenderer);

/**
 * Given an array of RGB values ( in [0,1] ), set the ambient and diffuse colors to that colors.
 *
 * The ambient color will be set such that the object will be rendered with exactly that color,
 * which can be useful to preserve data encoded in the color.
 *
 * @param tree_actor 		The actor to set the colors for.
 * @param rgb 				The RGB values [0-1] to set the colors to.
 */
void setColorsByEncoding(vtkNew<vtkActor> &tree_actor, const std::array<double, 3> &rgb);

/**
 * Given a TreeMeshes, build an actor collection with an actor for each part of the tree: trunk, leaves and fruit.
 *
 * Actors are colored according to color encoding.
 *
 * @param meshes 		The TreeMeshes to build the actor collection for.
 * @return 				The built actor collection.
 */
vtkNew<vtkActorCollection> buildTreeActors(const TreeMeshes& meshes);

/**
 * Given a SimplifiedOrchard, construct the required actors to visualize it.
 * Color encoding to identify different parts is used.
 *
 * @param orchard 		The SimplifiedOrchard to build the actors for.
 * @return 				The built actor collection.
 */
vtkNew<vtkActorCollection> buildOrchardActors(const SimplifiedOrchard &orchard);

/**
 * Build a vtkActor representing a simple flat ground plane, with GROUND_PLANE_RGB as color.
 *
 * @return 		The built vtkActor.
 */
vtkNew<vtkActor> buildGroundPlaneActor();

/**
 * Build a point cloud actor for a vtkDepthImageToPointCloud.
 *
 * @param depthToPointCloud 		The vtkDepthImageToPointCloud to build the actor for.
 * @return 							The built vtkActor.
 */
vtkNew<vtkActor> buildDepthImagePointCloudActor(vtkNew<vtkDepthImageToPointCloud> &depthToPointCloud);

#endif //NEW_PLANNERS_VTK_H