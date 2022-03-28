
#include <ompl/geometric/PathSimplifier.h>
#include <boost/range/irange.hpp>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "probe_retreat_move.h"
#include "experiment_utils.h"
#include "EndEffectorOnShellGoal.h"

ompl::geometric::PathGeometric retreat_travel_probe(
        std::shared_ptr<ompl::base::SpaceInformation> &si,
        OMPLSphereShellWrapper &shell,
        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
        size_t retreat_idx,
        size_t approach_idx,
        bool simplify) {

    ompl::geometric::PathGeometric appleToApple(approaches[retreat_idx].second);

    appleToApple.reverse();

    appleToApple.append(shell.path_on_shell(approaches[retreat_idx].first, approaches[retreat_idx].first));

    appleToApple.append(approaches[approach_idx].second);

    if (simplify) {
        ompl::geometric::PathSimplifier(si).simplifyMax(appleToApple);
    }

    return appleToApple;
}

ompl::geometric::PathGeometric planFullPath(
        std::shared_ptr<ompl::base::SpaceInformation> &si,
        ompl::base::State *start,
        OMPLSphereShellWrapper &shell,
        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches) {


    ompl::geometric::PathGeometric fullPath(si, start);
    for (size_t approachIdx: boost::irange<size_t>(0, approaches.size()-1)) {
        fullPath.append(retreat_travel_probe(si, shell, approaches, approachIdx, approachIdx + 1, true));
    }
    fullPath.interpolate();

    return fullPath;
}

std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> planApproaches(
        const std::vector<Apple> &apples_in_order,
        const ompl::base::OptimizationObjectivePtr &objective,
        OMPLSphereShellWrapper &shell,
        const std::shared_ptr<ompl::base::SpaceInformation> &si) {

    std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches;

    for (const Apple &apple: apples_in_order) {

        auto state_outside = shell.state_on_shell(apple);

        auto planner = std::make_shared<ompl::geometric::PRMstar>(si);
        if (auto approach = planFromStateToApple(*planner, objective, state_outside->get(), apple, 1.0)) {
            approaches.emplace_back(apple, *approach);
        }
    }

    return approaches;
}

void optimizeExit(const Apple &apple,
                  ompl::geometric::PathGeometric &path,
                  const ompl::base::OptimizationObjectivePtr &objective,
                  OMPLSphereShellWrapper &shell,
                  const std::shared_ptr<ompl::base::SpaceInformation> &si) {

    auto shellGoal = std::make_shared<EndEffectorOnShellGoal>(si, shell, apple.center);

    ompl::geometric::PathSimplifier simplifier(si, shellGoal);

    path.reverse();

    for (size_t i = 0; i < 10; ++i) {
        simplifier.findBetterGoal(path, 0.1);
        simplifier.simplify(path, 0.1);
    }

    path.reverse();

}
