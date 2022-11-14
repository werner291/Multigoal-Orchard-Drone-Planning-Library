
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ompl/util/RandomNumbers.h>
#include "../src/StreamingAlphaShape.h"

TEST(StreamingAlphaShape_test, test) {

	// Generate a set of three bounding boxes to ensure a somewhat non-convex set of data points.

	std::vector<Eigen::AlignedBox<double, 3>> boxes;

	boxes.emplace_back(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
	boxes.emplace_back(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(-1, -1, -1));

	// Then, generate a set of 10k points inside the bounding boxes.

	std::vector<Eigen::Vector3d> pts;

	ompl::RNG rng;

	for (int i = 0; i < 10000; i++) {
		const Eigen::AlignedBox<double, 3> &box = boxes[rng.uniformInt(0, (int) boxes.size() - 1)];

		pts.emplace_back(
				rng.uniformReal(box.min()[0], box.max()[0]),
				rng.uniformReal(box.min()[1], box.max()[1]),
				rng.uniformReal(box.min()[2], box.max()[2])
		);
	}

	// Stream them into the alpha shape, one by one.

	StreamingAlphaShape shape(0.1);

	for (const Eigen::Vector3d &pt : pts) {
		shape.addPoint(pt);
	}

	// Then, for all points, check that the alpha shape either contains them in is interior, or is on the boundary by a maximum margin.

}