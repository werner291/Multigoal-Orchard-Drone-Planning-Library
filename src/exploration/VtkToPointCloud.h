
#ifndef NEW_PLANNERS_VTKTOPOINTCLOUD_H
#define NEW_PLANNERS_VTKTOPOINTCLOUD_H

#include <vtkPolyData.h>
#include "SegmentedPointCloud.h"

/**
 * Extracts SegmentedPointCloud from vtkPolyData, classifying the colors.
 *
 * @param pPolyData 	vtkPolyData to extract SegmentedPointCloud from
 * @return 				SegmentedPointCloud
 */
SegmentedPointCloud segmentPointCloudData(vtkPolyData *pPolyData);

#endif //NEW_PLANNERS_VTKTOPOINTCLOUD_H
