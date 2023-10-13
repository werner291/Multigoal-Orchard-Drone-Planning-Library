#ifndef NEW_PLANNERS_GREATCIRCLEMETRIC_H
#define NEW_PLANNERS_GREATCIRCLEMETRIC_H

#include <Eigen/Geometry>

/**
 * Helper struct to calculate great-circle distance between two points,
 * under the context of a pre-given sphere somewhere in space.
 *
 * For points A,B and sphere center C, the result is simply the angle between CA and CB.
 */
class GreatCircleMetric {

	// The center of the sphere (radius not needed)
	const Eigen::Vector3d sphere_center;

public:
	/**
	 * Constructor.
	 * @param sphereCenter 		The center of the sphere.
	 */
	explicit GreatCircleMetric(Eigen::Vector3d sphereCenter);

	/**
	 * Calculate the great-circle distance between two points.
	 *
	 * @param a 	The first point.
	 * @param b 	The second point.
	 * @return 		The distance.
	 */
	[[nodiscard]] double measure(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;
};

#endif //NEW_PLANNERS_GREATCIRCLEMETRIC_H
