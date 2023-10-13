
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

#include "../experiment_utils/TreeMeshes.h"

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
 * Add the given actor collection to the given renderer.
 *
 * @param orchard_actors 		The actor collection to add to the renderer.
 * @param sensorRenderer 		The renderer to add the actor collection to.
 */
void addActorCollectionToRenderer(vtkActorCollection *orchard_actors, vtkRenderer *sensorRenderer);

/**
 * Convert a ROS shape_msgs::msg::Mesh to a vtkPolyData.
 * @param mesh 		The ROS mesh to convert.
 * @return 			The converted vtkPolyData.
 */
vtkNew<vtkPolyData> rosMeshToVtkPolyData(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkCellArray> meshTrianglesToVtkCells(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkPoints> meshVerticesToVtkPoints(const shape_msgs::msg::Mesh &mesh);

/**
 * Create a vtkActor with the given ROS mesh message.
 *
 * @param mesh 		The ROS mesh message to create the actor for.
 * @return 			The created vtkActor.
 */
vtkNew<vtkActor> createActorFromMesh(const shape_msgs::msg::Mesh &mesh);

vtkSmartPointer<vtkActor> addColoredMeshActor(const shape_msgs::msg::Mesh &mesh,
											  const std::array<double, 4> &color_rgba,
											  vtkRenderer *renderer,
											  bool visible = true);

vtkSmartPointer<vtkActor> createColoredMeshActor(const shape_msgs::msg::Mesh &mesh, const std::array<double, 4> &color_rgba, bool visible = true);

std::vector<vtkSmartPointer<vtkActor>> createColoredMeshActors(const std::vector<shape_msgs::msg::Mesh> &meshes,
															   const std::array<double, 4> &color_rgba,
															   bool visible = true);


/**
 * Build a simple window interactor that sets up a 33ms timer as well.
 * @param visualizerWindow 		The window to build the interactor for.
 * @return 						The created interactor.
 */
vtkNew<vtkRenderWindowInteractor> buildVisualizerWindowInteractor(vtkNew<vtkRenderWindow> &visualizerWindow);

// /**
//  * Build vtkPolyData from the given vector of ScanTargetPoint, in order to visualize them.
//  *
//  * The resulting poly data will have one point and one vertex for every ScanTargetPoint,
//  * colored dark blue, with matching indices.
//  *
//  * @param scan_targets 		The ScanTargetPoints to build the poly data for.
//  * @return 					The built vtkPolyData.
//  */
// vtkNew<vtkPolyData> mkVtkPolyDataFromScannablePoints(const std::vector<ScanTargetPoint> &scan_targets);

/**
 * Given an array of RGB values ( in [0,1] ), set the ambient and diffuse colors to that colors.
 *
 * The ambient color will be set such that the object will be rendered with exactly that color,
 * which can be useful to preserve data encoded in the color.
 *
 * @param tree_actor 		The actor to set the colors for.
 * @param rgb 				The RGB values [0-1] to set the colors to.
 */
void setColorsByEncoding(vtkNew<vtkActor> &tree_actor, const std::array<double, 3> &rgb, bool usePureColor);

/**
 * Given a TreeMeshes, build an actor collection with an actor for each part of the tree: trunk, leaves and fruit.
 *
 * Actors are colored according to color encoding.
 *
 * @param meshes 		The TreeMeshes to build the actor collection for.
 * @return 				The built actor collection.
 */
vtkNew<vtkActorCollection> buildTreeActors(const mgodpl::tree_meshes::TreeMeshes &meshes, bool usePureColor);

/**
 * Given a SimplifiedOrchard, construct the required actors to visualize it.
 * Color encoding to identify different parts is used.
 *
 * Note to the future: Ambient color in VTK appears to be very badly broken, so far the only thing that seems to work
 * is setting it on actor creation, so that's what we do. The ambient color of lights in the scene appears to simply
 * be ignored.
 *
 * Ideally, we'd just set up scene lighting to preserve the color of objects, but that doesn't seem to work.
 *
 * @param orchard 		The SimplifiedOrchard to build the actors for.
 * @param usePureColor 	Whether to maximize ambient color to preserve color encoding (see note above)
 * @return 				The built actor collection.
 */
vtkNew<vtkActorCollection> buildOrchardActors(const mgodpl::tree_meshes::SimplifiedOrchard &orchard, bool usePureColor);

/**
 * Build a vtkActor representing a simple flat ground plane, with GROUND_PLANE_RGB as color.
 *
 * @return 		The built vtkActor.
 */
vtkNew<vtkActor> buildGroundPlaneActor(bool usePureColor);

/**
 * Build a point cloud actor for a vtkDepthImageToPointCloud.
 *
 * @param depthToPointCloud 		The vtkDepthImageToPointCloud to build the actor for.
 * @return 							The built vtkActor.
 */
vtkNew<vtkActor> buildDepthImagePointCloudActor(vtkAlgorithmOutput *pointCloudInput);


class VtkPointCloudVisualization {

public:
	vtkNew<vtkPolyData> visitOrderVisualizationData;
	vtkNew<vtkPolyDataMapper> visitOrderVisualizationMapper;
	vtkNew<vtkActor> visitOrderVisualizationActor;

	VtkPointCloudVisualization(float r, float g, float b);

	vtkActor *getActor();

	void updatePoints(const std::vector<mgodpl::math::Vec3d> &points);
};

#endif //NEW_PLANNERS_VTK_H
