
#ifndef NEW_PLANNERS_STREAMINGALPHASHAPE_H
#define NEW_PLANNERS_STREAMINGALPHASHAPE_H


#include <variant>
#include "HashedSpatialIndex.h"

class StreamingAlphaShape {

	struct SinglePoint {
		Eigen::Vector3d point;
	};

	struct FromTriangle {

	};

	using TriggerSphere = std::variant<SinglePoint, FromTriangle>;

	HashedSpatialIndex<TriggerSphere> spatialIndex;

	void addPoint(const Eigen::Vector3d &point) {

		if (auto nearby = spatialIndex.any_within(point)) {

		} else {
			spatialIndex.insert(point, SinglePoint{point});
		}

	}

};


#endif //NEW_PLANNERS_STREAMINGALPHASHAPE_H
