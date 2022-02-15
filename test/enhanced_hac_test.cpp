#include <gtest/gtest.h>
#include <boost/range/irange.hpp>
#include <ompl/util/RandomNumbers.h>
#include <json/value.h>
#include <fstream>
#include "../src/agglomerative_clustering.h"
#include "../src/json_utils.h"

Json::Value toJSON(const agglomerative_clustering::AgglomerativeClustering<double>::TreeNode& result) {
    Json::Value val;

    val["representative"] = result.representative;
    if (result.children) {
        val["children"][0] = toJSON(*(result.children->first));
        val["children"][1] = toJSON(*(result.children->second));
    }

    return val;
}

TEST(EnhancedHACTest, hac_test) {

    std::vector<double> points;
    ompl::RNG rng;

    for (size_t i : boost::irange(0,1000)) {

        points.push_back(rng.uniformReal(-10.0, -9.0));
        points.push_back(rng.uniformReal(  9.0, 10.0));

    }

    using namespace agglomerative_clustering;

    size_t expensive_calls = 0;

    // 1079150
    // 4000000

    AgglomerativeClustering<double>::ExpensiveDistanceFn distance = [&](double a, double b) -> AgglomerativeClustering<double>::DistanceResult {
            expensive_calls += 1;
            return {abs(a-b),(a+b)/2.0};
    };

    AgglomerativeClustering<double> clustering(distance, points, 20);

    AgglomerativeClustering<double>::TreeNode result = clustering.run();

    std::cout << "Expensive calls: " << expensive_calls << " (n^2 = " << std::pow(points.size(),2) << ")" << std::endl;

    std::ofstream ofs("analysis/ehac.json");
    ofs << toJSON(result);
    ofs.close();

}