
#ifndef NEW_PLANNERS_SIMULATEDSENSOR_H
#define NEW_PLANNERS_SIMULATEDSENSOR_H

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkDepthImageToPointCloud.h>
#include <Eigen/Geometry>
#include <vtkRendererSource.h>
#include <optional>

#include "../exploration/SegmentedPointCloud.h"

/**
 * A simulated sensor that can be used to get point clouds from a vtk environment.
 */
struct SimulatedSensor {
	/// The renderer to get the point cloud from.
	vtkSmartPointer<vtkRenderer> sensorRenderer;
	/// The render window to render the scene to.
	vtkSmartPointer<vtkRenderWindow> sensorWindow;

	/// The depth to point cloud filter.
	vtkNew<vtkDepthImageToPointCloud> depthToPointCloud;

	vtkNew<vtkRendererSource> rendererSource;

	/**
	 * Build a simulated sensor.
	 *
	 * @param showWindow 		Whether to show the point-of-view of the sensor in a user-visible window.
	 */
	SimulatedSensor(bool showWindow = true);

	void addActorCollection(vtkActorCollection* actors);

	void addActor(vtkActor* actor);

	[[nodiscard]] vtkAlgorithmOutput* getPointCloudOutputPort() const;

	[[nodiscard]] vtkPolyData* getPointCloud() const;

	/**
	 * Render a snapshot of the scene from the given sensor pose, returning the segmented point cloud.
	 *
	 * @param from_pose 		The pose to render the scene from.
	 * @return 					The segmented point cloud.
	 */
	SegmentedPointCloud::ByType renderSnapshot(const Eigen::Isometry3d& from_pose);

};

#endif //NEW_PLANNERS_SIMULATEDSENSOR_H
