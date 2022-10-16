
#include <gtest/gtest.h>
#include <ompl/base/spaces/EmptyStateSpace.h>
#include <thread>
#include <ompl/geometric/PathGeometric.h>

#include "../src/TimedCostConvergenceTerminationCondition.h"

TEST(TimedCostConvergenceTerminationCondition_test, test0) {

	auto ss = std::make_shared<ompl::base::RealVectorStateSpace>(1);
	auto si = std::make_shared<ompl::base::SpaceInformation>(ss);

	ompl::base::ProblemDefinition pdef(si);

	TimedConvergenceTerminationCondition tcc(pdef, ompl::time::seconds(0.1), true);

	EXPECT_FALSE(tcc);

	std::this_thread::sleep_for(ompl::time::seconds(0.2));

	EXPECT_FALSE(tcc);

	std::this_thread::sleep_for(ompl::time::seconds(0.05));

	pdef.getIntermediateSolutionCallback()(nullptr, {}, ompl::base::Cost(1.0));

	EXPECT_FALSE(tcc);

	std::this_thread::sleep_for(ompl::time::seconds(0.05));

	EXPECT_FALSE(tcc);

	std::this_thread::sleep_for(ompl::time::seconds(0.1));

	EXPECT_TRUE(tcc);

}