#include <utility>
#include "../src/DronePathLengthObjective.h"
#include "greatcircle.h"

GreatCircleMetric::GreatCircleMetric(Eigen::Vector3d sphereCenter) : sphere_center(std::move(sphereCenter)) {}

double GreatCircleMetric::measure(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {

    Eigen::Vector3d na = (a-sphere_center).normalized();
    Eigen::Vector3d nb = (b-sphere_center).normalized();

    return acos(na.dot(nb));

}