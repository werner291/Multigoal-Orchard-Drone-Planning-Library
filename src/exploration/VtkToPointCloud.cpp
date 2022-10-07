
#include "VtkToPointCloud.h"
#include "ColorEncoding.h"

#include <vtkPointData.h>

SegmentedPointCloud segmentPointCloudData(vtkPolyData *pPolyData) {

	// Extract the position of the points
	auto points = pPolyData->GetPoints();
	// Extract the colors of the points
	auto colors = pPolyData->GetPointData()->GetScalars();

	// Quick sanity check to make sure the number of points and colors match
	assert(points->GetNumberOfPoints() == colors->GetNumberOfTuples());
	// ... and that every color has 3 components (RGB)
	assert(colors->GetNumberOfComponents() == 3);

	// Create a SegmentedPointCloud
	SegmentedPointCloud segmentedPointCloud;

	// Iterate over all points
	for (int i = 0; i < points->GetNumberOfPoints(); i++) {

		SegmentedPointCloud::Point point;

		// Extract the position of the point
		points->GetPoint(i, point.position.data());

		// Extract the color of the point
		Eigen::Vector3d color;
		colors->GetTuple(i, color.data());

		// Classify the color
		auto ptType = pointTypeByColor(color);

		// If the color is not classified, skip the point
		if (ptType.has_value()) {
			point.type = *ptType;
			// Otherwise, add the point to the SegmentedPointCloud
			segmentedPointCloud.points.push_back(point);
		}
	}

	// Return the SegmentedPointCloud
	return std::move(segmentedPointCloud);
}
