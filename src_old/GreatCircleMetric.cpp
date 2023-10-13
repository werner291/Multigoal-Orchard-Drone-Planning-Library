#include "GreatCircleMetric.h"

GreatCircleMetric::GreatCircleMetric(Eigen::Vector3d sphereCenter) : sphere_center(std::move(sphereCenter)) {}

double GreatCircleMetric::measure(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {

	// Compute the CA and CB vectors.
    Eigen::Vector3d na = (a-sphere_center).normalized();
    Eigen::Vector3d nb = (b-sphere_center).normalized();

	// Compute the angle between the two vectors.
    return acos(na.dot(nb));

}