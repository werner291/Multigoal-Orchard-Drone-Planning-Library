#include <variant>
#include "ScannablePointsIndex.h"
#include "HashedSpatialIndex.h"

bool ScannablePointsIndex::GNATEntry::operator==(const ScannablePointsIndex::GNATEntry &other) const {
	return point.point == other.point.point;
}

bool ScannablePointsIndex::GNATEntry::operator!=(const ScannablePointsIndex::GNATEntry &other) const {
	return !(*this == other);
}

ScannablePointsIndex::ScannablePointsIndex(const std::vector<ScanTargetPoint> &in_points) {

	fruitSurfacePoints.setDistanceFunction([&](const GNATEntry &a, const GNATEntry &b) {
		return (a.point.point - b.point.point).norm();
	});

	for (size_t i = 0; i < in_points.size(); i++) {
		GNATEntry entry;
		entry.point = in_points[i];
		entry.backref = i;
		fruitSurfacePoints.add(entry);
	}

}

std::vector<size_t> ScannablePointsIndex::findScannedPoints(const Eigen::Vector3d &scanSource,
															const Eigen::Vector3d &scanDirection,
															const double fov,
															double maxDistance,
															const std::vector<SegmentedPointCloud::TargetPoint>& points) const {

	std::vector<GNATEntry> near_sensor = findNearSensor(scanSource, maxDistance);

	HashedSpatialIndex<std::monostate> scannedPointsIndex(VISIBLE_POINT_MAX_DISTANCE, 1000);

	for (const auto& point : points) {
		scannedPointsIndex.insert(point.position, {});
	}

	std::vector<size_t> scanned;

	for (auto& entry: near_sensor) {

		// Check if the point is within the field of view of the sensor
		Eigen::Vector3d pointDirection = entry.point.point - scanSource;
		double angle = std::acos(pointDirection.dot(scanDirection) / (pointDirection.norm() * scanDirection.norm()));
		if (angle > fov / 2) {
			continue;
		}

		// Check if the point's normal is turned towards the sensor
		if (entry.point.normal.dot(scanDirection) < 0) {
			continue;
		}

		// Check if a target point is near this place.
		if (!scannedPointsIndex.any_within(entry.point.point, VISIBLE_POINT_MAX_DISTANCE).has_value()) {
			continue;
		}

		scanned.push_back(entry.backref);
	}

	return scanned;
}

std::vector<ScannablePointsIndex::GNATEntry>
ScannablePointsIndex::findNearSensor(const Eigen::Vector3d &scanSource, double maxDistance) const {
	std::vector<GNATEntry> near_sensor;
	GNATEntry dummy_entry { ScanTargetPoint { scanSource }, 0 };
	fruitSurfacePoints.nearestR(dummy_entry, maxDistance, near_sensor);
	return near_sensor;
}
