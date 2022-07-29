
#pragma once

#include <ompl/geometric/PathSimplifier.h>
#include <boost/range/irange.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "EndEffectorOnShellGoal.h"
#include "general_utilities.h"



ompl::geometric::PathGeometric optimize(const ompl::geometric::PathGeometric& path,
                                        const ompl::base::OptimizationObjectivePtr &objective,
                                        const std::shared_ptr<ompl::base::SpaceInformation> &si);

template<typename ShellPoint>
ompl::geometric::PathGeometric optimizeExit(const ompl::base::Goal* goal,
                                            ompl::geometric::PathGeometric path,
                                            const ompl::base::OptimizationObjectivePtr &objective,
                                            const OMPLSphereShellWrapper<ShellPoint> &shell,
                                            const std::shared_ptr<ompl::base::SpaceInformation> &si) {

	auto shellGoal = std::make_shared<EndEffectorOnShellGoal<ShellPoint>>(si, shell, shell.project(goal));

	ompl::geometric::PathSimplifier simplifier(si, shellGoal);

	path.reverse();

	ompl::geometric::PathGeometric backup = path;
	for (size_t i = 0; i < 10; ++i) {
		simplifier.findBetterGoal(path, 0.1);
		simplifier.simplify(path, 0.1);

		if (backup.length() < path.length()) {
			path = backup;
		}
	}

	path.reverse();

	return path;

}