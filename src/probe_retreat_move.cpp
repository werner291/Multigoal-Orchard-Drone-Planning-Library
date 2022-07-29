
#include <ompl/geometric/PathSimplifier.h>
#include <boost/range/irange.hpp>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/for_each.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "probe_retreat_move.h"
#include "EndEffectorOnShellGoal.h"
#include "general_utilities.h"

ompl::geometric::PathGeometric optimize(const ompl::geometric::PathGeometric& path,
                                        const ompl::base::OptimizationObjectivePtr &objective,
                                        const std::shared_ptr<ompl::base::SpaceInformation> &si) {

    ompl::geometric::PathGeometric new_path(path);

    ompl::geometric::PathSimplifier simplifier(si);
    simplifier.simplifyMax(new_path);

    if (path.length() < new_path.length()) {
        std::cout << "Optimization failed: " << path.length() << " vs " << new_path.length() << std::endl;
        return path;
    } else {
        return new_path;
    }
}
