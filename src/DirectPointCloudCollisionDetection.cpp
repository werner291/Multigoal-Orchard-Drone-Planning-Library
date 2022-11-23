
#include <geometric_shapes/shapes.h>
#include "DirectPointCloudCollisionDetection.h"

bool DirectPointCloudCollisionDetection::checkCollision(const moveit::core::RobotState &state) const {

	// Extract all collision shapes and their transforms, and check for collisions one-by-one.

	return std::any_of(state.getRobotModel()->getLinkModelsWithCollisionGeometry().begin(),
				state.getRobotModel()->getLinkModelsWithCollisionGeometry().end(),
				[&](const moveit::core::LinkModel* lm) {
					auto shape = lm->getShapes()[0];
					auto transform = lm->getCollisionOriginTransforms()[0];
					const auto& tf = state.getGlobalLinkTransform(lm);
					auto total_tf = tf * transform;
					return checkCollision(shape, total_tf);
				});
}

bool DirectPointCloudCollisionDetection::checkCollision(const shapes::ShapeConstPtr &shape, const Eigen::Isometry3d &pose) const {

	// Split by shape type and call the method again.

	switch (shape->type) {
		case shapes::ShapeType::BOX:
			return checkCollision(*std::static_pointer_cast<const shapes::Box>(shape), pose);
		default:
			throw std::runtime_error("Unsupported shape type");
	}

}

void DirectPointCloudCollisionDetection::addPoints(std::vector<Eigen::Vector3d> points) {

	// Delete all points that are already close to another point in the tree

	std::vector<Eigen::Vector3d> filtered_points;

	for (auto & point : points) {
		NN_incremental_search search(tree, Point(point.x(), point.y(), point.z()));
		// TODO could do some kinda LOD thing?
		if (search.begin() == search.end() || search.begin()->second > 0.05) {
			filtered_points.push_back(point);
		}
	}

	// Add all points to the tree.
	for (const auto &point : filtered_points) {
		tree.insert(Point(point.x(), point.y(), point.z()));
	}

	if (!filtered_points.empty()) {
		version += 1;
	}

}

bool DirectPointCloudCollisionDetection::checkCollision(const shapes::Box &shape, const Eigen::Isometry3d &pose) const {

	// Get the bounding sphere.
	auto radius = Eigen::Vector3d(shape.size).norm() / 2;

	// Get the center of the bounding sphere.
	Eigen::Vector3d center = pose.translation();

	Eigen::Isometry3d inv_pose = pose.inverse();

	// Perform a spherical range query.

	NN_incremental_search search(tree, Point(center.x(), center.y(), center.z()));

	// Iterate on the points one by one until we either find a collision or find a point outside the sphere.

	for (NN_iterator it = search.begin(); it != search.end() && it->second <= radius; ++it) {

		Eigen::Vector3d point(it->first.x(), it->first.y(), it->first.z());
		Eigen::Vector3d transformed_point = inv_pose * point;

		if (std::abs(transformed_point.x()) < shape.size[0] / 2 &&
			std::abs(transformed_point.y()) < shape.size[1] / 2 &&
			std::abs(transformed_point.z()) < shape.size[2] / 2) {
			return true;
		}
	}

	return false;

}

bool DirectPointCloudCollisionDetection::checkCollisionInterpolated(const moveit::core::RobotState &state1,
																	const moveit::core::RobotState &state2,
																	double maxStep) const {

	moveit::core::RobotState interpolated_state(state1.getRobotModel());

	double distance = state1.distance(state2);

	size_t n_steps = std::max((size_t) 1,(size_t) std::ceil(distance / maxStep));

	// TODO: Possible optimization: work with upper/lower margins, similar to how spherical ray marching works.

	for (size_t step_i = 0; step_i <= n_steps; step_i++) {
		double t = (double) step_i / (double) n_steps;
		// Starting interpolation at the far end, hoping I get a collision sooner if there is one.
		state2.interpolate(state1, t, interpolated_state);
		interpolated_state.update();

		if (checkCollision(interpolated_state)) {
			return true;
		}
	}

	return false;

}

bool DirectPointCloudCollisionDetection::checkCollisionInterpolated(const RobotPath &path, double maxStep) const {
	for (size_t waypoint_i = 0; waypoint_i + 1 < path.waypoints.size(); waypoint_i++) {

		const auto &before = path.waypoints[waypoint_i];
		const auto &after = path.waypoints[waypoint_i + 1];

		const double MAX_STEP = 0.2;

		if (checkCollisionInterpolated(before, after, MAX_STEP)) {
			return true;
		}
	}
	return false;
}

size_t DirectPointCloudCollisionDetection::getVersion() const {
	return version;
}
