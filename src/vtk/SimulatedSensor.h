
#ifndef NEW_PLANNERS_SIMULATEDSENSOR_H
#define NEW_PLANNERS_SIMULATEDSENSOR_H

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkDepthImageToPointCloud.h>
#include <Eigen/Geometry>

/**
 * A simulated sensor that can be used to get point clouds from a vtk environment.
 */
struct SimulatedSensor {
	/// The renderer to get the point cloud from.
	vtkSmartPointer<vtkRenderer> sensorRenderer;
	/// The render window to render the scene to.
	vtkSmartPointer<vtkRenderWindow> sensorWindow;
	/// The depth to point cloud filter.
	vtkSmartPointer<vtkDepthImageToPointCloud> depthToPointCloud;

	/**
	 * Build a simulated sensor.
	 */
	SimulatedSensor();

	void addActorCollection(vtkActorCollection* actors);

	void addActor(vtkActor* actor);

	[[nodiscard]] vtkAlgorithmOutput* getPointCloudOutputPort() const;

	[[nodiscard]] vtkPolyData* getPointCloud() const;

	void requestRender(const Eigen::Isometry3d& from_pose);
};

#endif //NEW_PLANNERS_SIMULATEDSENSOR_H
