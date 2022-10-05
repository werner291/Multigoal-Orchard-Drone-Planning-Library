
#ifndef NEW_PLANNERS_SCANNABLEPOINTSINDEX_H
#define NEW_PLANNERS_SCANNABLEPOINTSINDEX_H

#include <Eigen/Core>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include "exploration/SegmentedPointCloud.h"
#include "exploration/scan_points.h"

static const double VISIBLE_POINT_MAX_DISTANCE = 0.01;

/**
 * A spatial index of the points that need to be scanned.
 */
class ScannablePointsIndex {

	/**
	 * An entry into NearestNeighborsGNAT, containing a backreference to the original point's index in the vector.
	 */
	struct GNATEntry {
		ScanTargetPoint point;
		size_t backref;

		bool operator==(const GNATEntry &other) const;

		bool operator!=(const GNATEntry &other) const;
	};

	/// The spatial index of the points that need to be scanned
	ompl::NearestNeighborsGNAT<GNATEntry> fruitSurfacePoints;

	/**
	 * Find all points that are within a certain distance of the sensor.
	 *
	 * @param scanSource 		The position of the sensor
	 * @param maxDistance 		The maximum distance of the sensor
	 * @return 					A vector of all points that are within the sensor's range
	 */
	std::vector<GNATEntry> findNearSensor(const Eigen::Vector3d &scanSource, double maxDistance) const;

public:

	/**
	 * Create a new ScannablePointsIndex from a vector of points.
	 * This vector is assumed to remain valid for the lifetime of the spatial index,
	 * as the index will be returning indices into this vector.
	 *
	 * @param in_points 		The points that need to be scanned
	 */
	explicit ScannablePointsIndex(const std::vector<ScanTargetPoint>& in_points);

	/**
	 * Find all points that are within the field of view of the sensor and are turned towards the sensor,
	 * within a given maximum distance form the sensor.
	 *
	 * @param scanSource 		The position of the sensor
	 * @param scanDirection 	The direction of the sensor
	 * @param fov 				The field of view of the sensor (in radians, max angle from the sensor's direction)
	 * @param maxDistance 		The maximum distance from the sensor
	 *
	 * Note that this method does not explicitly handle occlusions, though if the distances are small enough,
	 * it should not matter.
	 *
	 * Alternatively, perhaps some kind of line segment collision detection could be used?
	 * Or perhaps the sensor's point cloud? (I did try the latter once but it was too slow.)
	 */
	std::vector<size_t> findScannedPoints(const Eigen::Vector3d &scanSource,
										  const Eigen::Vector3d &scanDirection,
										  double fov,
										  double maxDistance,
										  const SegmentedPointCloud& visiblePoints) const;


};

#endif //NEW_PLANNERS_SCANNABLEPOINTSINDEX_H
