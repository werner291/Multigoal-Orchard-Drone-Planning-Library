
#include <gtest/gtest.h>
#include <Eigen/Core>

#include "../src/OnlineOrderOptimization.h"
#include "../src/AnytimeOptimalInsertion.h"

TEST(OnlineOrderOptimization, test) {

	// Generate 1000 points at random.

	std::vector<Eigen::Vector3d> pts;

	ompl::RNG rng;

	for (int i = 0; i < 1000; i++) {
		pts.push_back(Eigen::Vector3d(rng.uniformReal(-10, 10), rng.uniformReal(-10, 10), rng.uniformReal(-10, 10)));
	}

	std::vector<size_t> optimal;
	double old_size;

	bool improved = false;

	// Create an AnytimeOptimalInsertion object.
	AnytimeOptimalInsertion<size_t> opt(
			[&pts](size_t i) { return pts[i].norm(); },
			[&pts](size_t i, size_t j) { return (pts[i] - pts[j]).norm(); },
			[&](const std::vector<size_t> &order) {

				double new_size = 0;

				for (size_t i = 1; i < order.size(); i++) {
					new_size += (pts[order[i]] - pts[order[i-1]]).norm();
				}

				EXPECT_LT(new_size, old_size);

				old_size = new_size;
				optimal = order;

				improved = true;

			}
	);

	optimal = opt.getVisitOrdering();
	for (size_t i = 1; i < optimal.size(); i++) {
		old_size += (pts[optimal[i]] - pts[optimal[i-1]]).norm();
	}



}