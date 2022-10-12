
#include "SegmentedPointCloud.h"

std::vector<Eigen::Vector3d> isolateLeafPoints(const SegmentedPointCloud &segmentedPointCloud) {
	std::vector<Eigen::Vector3d> points;

	for (const auto &item: segmentedPointCloud.points) {
		if (item.type == SegmentedPointCloud::PT_SOFT_OBSTACLE && item.position.z() > 1.0e-6) {
			points.push_back(item.position);
		}
	}

	return points;
}
