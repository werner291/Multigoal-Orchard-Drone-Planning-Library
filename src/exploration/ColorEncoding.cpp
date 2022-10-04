
#include "ColorEncoding.h"

/**
 * Checks if the specified color component matches the specified color.
 *
 * @param color 		Color to check		(RGB in [0,255])
 * @param rgb 			Color to compare to (RGB in [0,1])
 * @param component 	Component to check (0 = R, 1 = G, 2 = B)
 * @return 				True if the color component matches the specified color, false otherwise
 */
bool isComponentSame(const Eigen::Vector3d &color, const std::array<double, 3> &rgb, int component) {
	return abs(color[component] - floor(255.0 * rgb[component])) < 1.0e-6;
}

/**
 * Checks if the specified color matches the specified color.
 *
 * @param color 		Color to check		(RGB in [0,255])
 * @param rgb 			Color to compare to (RGB in [0,1])
 * @return 				True if the color matches the specified color, false otherwise
 */
bool isColorSame(const Eigen::Vector3d &color, const std::array<double, 3> &rgb) {
	return isComponentSame(color, rgb, 0)
		&& isComponentSame(color, rgb, 1)
		&& isComponentSame(color, rgb, 2);
}

std::optional<PointType> pointTypeByColor(const Eigen::Vector3d &color) {

	// Simply check if the color matches any of the encoded colors

	if (isColorSame(color, GROUND_PLANE_RGB) || isColorSame(color, TRUNK_RGB)) {
		// Ground and trunk are hard obstacles
		return PT_OBSTACLE;
	} else if (isColorSame(color, FRUIT_RGB)) {
		// Fruit is a target
		return PT_TARGET;
	} else if (isColorSame(color, LEAVES_RGB)) {
		// Leaves are soft obstacles
		return PT_SOFT_OBSTACLE;
	} else {
		// Unknown color (probably background)
		return std::nullopt;
	}

}
