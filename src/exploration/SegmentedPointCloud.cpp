
#include "SegmentedPointCloud.h"

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
