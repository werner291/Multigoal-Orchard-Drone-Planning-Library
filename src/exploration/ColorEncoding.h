
#ifndef NEW_PLANNERS_COLORENCODING_H
#define NEW_PLANNERS_COLORENCODING_H

#include <array>
#include <Eigen/Core>
#include <optional>
#include "SegmentedPointCloud.h"

/// Color of the ground plane (assuming no shading/lighting)
/// Note that colors may need to be multiplied into the [0, 255] range and/or rounded.
const std::array<double, 3> GROUND_PLANE_RGB = {0.3, 0.2, 0.1};
/// Color of the trunk of the tree (assuming no shading/lighting)
const std::array<double, 3> FRUIT_RGB = {1.0, 0.0, 0.0};
/// Color of the trunk of the tree (assuming no shading/lighting)
const std::array<double, 3> TRUNK_RGB = {0.5, 0.3, 0.1};
/// Color of the leaves of the tree (assuming no shading/lighting)
const std::array<double, 3> LEAVES_RGB = {0.0, 1.0, 0.0};

/**
 * Extracts point type from color (we intentionally engineer the color encoding to be easy to decode).
 *
 * Note that, since background is not one of the encoded colors, it will return an empty optional.
 *
 * @param color 		Color of the point
 * @return 				Point type, if the color is recognized. Otherwise, std::nullopt.
 */
std::optional<PointType> pointTypeByColor(const Eigen::Vector3d &color);

#endif //NEW_PLANNERS_COLORENCODING_H
