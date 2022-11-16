
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

SegmentedPointCloud::ByType SegmentedPointCloud::split_by_type() const {
	ByType result;
	for (const auto& p : points) {
		switch (p.type) {
			case PT_OBSTACLE:
				result.obstacle.push_back(p.position);
				break;
			case PT_SOFT_OBSTACLE:
				result.soft_obstacle.push_back(p.position);
				break;
			case PT_TARGET:
				result.target.push_back(p.position);
				break;
		}
	}
	return result;
}
