
/*
 * File containing code for relating to VTK stuff that is specific to the orchard visualization.
 */

#ifndef NEW_PLANNERS_VISUALIZATIONSPECIFICS_H
#define NEW_PLANNERS_VISUALIZATIONSPECIFICS_H

#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include "../exploration/scan_points.h"
#include "../StreamingConvexHull.h"
#include "../TreeMeshes.h"
#include "VtkRobotModel.h"
#include "SimulatedSensor.h"
#include "../exploration/DynamicMeshHullAlgorithm.h"

/**
 * A wrapper around the Vtk stuff necessary to visualize the set of scannable/scanned points.
 */
struct FruitSurfaceScanTargetsActor {
	vtkNew<vtkPolyData> fruitSurfacePolyData;
	vtkNew<vtkActor> fruitSurfacePointsActor;

	/**
	 * Constructor.
	 * @param pt 	The points to visualizer; order is important as it will be used later to update the color.
	 */
	explicit FruitSurfaceScanTargetsActor(const std::vector<ScanTargetPoint>& pt);

	/**
	 * Given a vector of points that were just scanned, update the color of the points to reflect that.
	 *
	 * @param indices 		The indices of the points that were just scanned; indices must match the vector passed to the constructor.
	 */
	void markAsScanned(const std::vector<size_t> &indices);
};


/**
 * A wrapper around the Vtk stuff necessary to visualize the StreamingConvexHull.
 */
struct ConvexHullActor {

	vtkNew<vtkActor> actor;
	vtkNew<vtkPolyData> polyData;
	vtkNew<vtkPolyDataMapper> mapper;

	ConvexHullActor();

	/**
	 * Extract the current convex hull from the StreamingConvexHull and update the actor.
	 *
	 * @param convexHull 		The StreamingConvexHull to extract the convex hull from.
	 */
	void update(const shape_msgs::msg::Mesh &mesh);
};

/**
 * Build a SimulatedSensor with the specific contents of the given workspace.
 *
 * @param orchard 						The orchard to build the viewer for.
 * @param robotModel 					The robot model to build the viewer for.
 * @return 						        The SimulatedSensor.
 */
SimulatedSensor buildSensorSimulator(const SimplifiedOrchard &orchard, VtkRobotmodel &robotModel);

std::vector<Eigen::Vector3d> extractEndEffectorTrace(const robot_trajectory::RobotTrajectory &trajectory);

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> extractTargetHullPointsSegments(const DynamicMeshHullAlgorithm &dbsa);

std::vector<Eigen::Vector3d> extractVisitOrderPoints(const DynamicMeshHullAlgorithm &dbsa, Eigen::Isometry3d &eePose);

#endif //NEW_PLANNERS_VISUALIZATIONSPECIFICS_H
