// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "MinimumClearanceOctree.h"
#include "../utilities/math_utils.h"

MinimumClearanceOctree::MinimumClearanceOctree(const Eigen::AlignedBox3d &bounds, double minLeafSize)
		: bounds(bounds), min_leaf_size(minLeafSize) {
}

void
MinimumClearanceOctree::update(const std::function<std::pair<double, Eigen::Vector3d>(const Eigen::Vector3d &)> &signedDistanceFunction) {

	updateInternal(signedDistanceFunction, root, this->bounds);

}

std::pair<double, Eigen::Vector3d>
MinimumClearanceOctree::getSignedDistanceAndGradient(const Eigen::Vector3d &point) const {

	Eigen::AlignedBox3d node_bounds = this->bounds;
	const Node *node = &root;

	while (!node->children->empty()) {

		auto octant = OctantIterator(node_bounds);

		for (size_t i = 0; i < 8; i++) {
			if ((*octant).contains(point)) {
				node = &(*node->children)[i];
				node_bounds = *octant;
				break;
			}
		}

	}

	Eigen::Vector3d cell_center = node_bounds.center();

	return {node->minimum_distance_at_center + (point - cell_center).dot(node->gradient_at_center),
			node->gradient_at_center};

}

void
MinimumClearanceOctree::updateInternal(const std::function<std::pair<double, Eigen::Vector3d>(const Eigen::Vector3d &)> &signedDistanceFunction,
									   MinimumClearanceOctree::Node &node,
									   const Eigen::AlignedBox3d &node_bounds) {

	Eigen::Vector3d cell_center = node_bounds.center();

	const auto &[distance_at_center, gradient_at_center] = signedDistanceFunction(cell_center);


	if (node.minimum_distance_at_center < distance_at_center) {
		node.minimum_distance_at_center = distance_at_center;
		node.gradient_at_center = gradient_at_center;
	}

	double half_extent_magnitude = (node_bounds.max() - node_bounds.min()).norm() / 2.0;

	if (half_extent_magnitude < min_leaf_size) {
		return;
	}

	if (abs(node.minimum_distance_at_center) > half_extent_magnitude) {
		node.children.reset();
	} else {
		if (!node.children) {
			node.children = std::make_unique<std::array<Node, 8>>();
		}

		auto octant = OctantIterator(node_bounds);

		for (size_t i = 0; i < 8; i++) {
			updateInternal(signedDistanceFunction, (*node.children)[i], *(octant++));
		}

	}


}

const Eigen::AlignedBox3d &MinimumClearanceOctree::getBounds() const {
	return bounds;
}

double MinimumClearanceOctree::getMinLeafSize() const {
	return min_leaf_size;
}

const MinimumClearanceOctree::Node &MinimumClearanceOctree::getRoot() const {
	return root;
}

std::vector<Eigen::Vector3d> implicitSurfacePoints(const MinimumClearanceOctree &octree) {

	std::vector<std::pair<const MinimumClearanceOctree::Node *, Eigen::AlignedBox3d>> stack{
			{&octree.getRoot(), octree.getBounds()}};

	std::vector<Eigen::Vector3d> points;

	while (!stack.empty()) {

		auto [node, bounds] = stack.back();
		stack.pop_back();

		if (!node->children) {
			points.emplace_back(
					bounds.center() - node->gradient_at_center.normalized() * node->minimum_distance_at_center);
		} else {
			auto octant = OctantIterator(bounds);
			for (size_t i = 0; i < 8; i++) {
				stack.emplace_back(&(*node->children)[i], *(octant++));
			}
		}
	}

	return points;

}
