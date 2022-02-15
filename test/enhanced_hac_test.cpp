
#include "../src/agglomerative_clustering.h"

TEST(EnhancedHACTest, hac_test) {

    std::vector<double> points;
    ompl::RNG rng;

    for (size_t i : boost::irange(0,1000)) {

        points.push_back(rng.uniformReal(-10.0, -9.0));
        points.push_back(rng.uniformReal(  9.0, 10.0));

    }

    AgglomerativeClustering<double> aggl(
            []()
            );

}