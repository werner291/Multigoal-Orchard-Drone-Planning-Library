
#include <ompl/geometric/PathSimplifier.h>
#include <boost/range/irange.hpp>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/for_each.hpp>
#include "probe_retreat_move.h"
#include "experiment_utils.h"
#include "EndEffectorOnShellGoal.h"
#include "general_utilities.h"

ompl::geometric::PathGeometric retreat_travel_probe(
        std::shared_ptr<ompl::base::SpaceInformation> &si,
        OMPLSphereShellWrapper &shell,
        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
        size_t retreat_idx,
        size_t approach_idx,
        bool simplify) {

    ompl::geometric::PathGeometric appleToApple(approaches[retreat_idx].second);

    appleToApple.reverse();
// FIXME bug here?
    const ompl::geometric::PathGeometric &pathOnShell = shell.path_on_shell(approaches[retreat_idx].first,
                                                                            approaches[approach_idx].first);

    std::cout << "Shell path length:" << pathOnShell.length() << std::endl;

    appleToApple.append(pathOnShell);

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
        const ompl::geometric::PathGeometric &retreatTravelProbe = retreat_travel_probe(si, shell, approaches,
                                                                                        approachIdx, approachIdx + 1,
                                                                                        true);
        
        std::cout << "RTP path length:" << retreatTravelProbe.length() << std::endl;
        
        fullPath.append(retreatTravelProbe);
    }
//    fullPath.interpolate();

    return fullPath;
}

//std::vector<std::pair<Apple, ompl::geometric::PathGeometric>>
//planApproaches(const std::vector<Apple> &apples_in_order, const ompl::base::OptimizationObjectivePtr &objective,
//               OMPLSphereShellWrapper &shell, const std::shared_ptr<ompl::base::SpaceInformation> &si, bool simplify) {
//
//    return apples_in_order
//        | ranges::views::for_each([&](const Apple& apple) { return planApproachForApple(apple, objective, shell, si, simplify);})
//        | ranges::to_vector;
//}

ompl::geometric::PathGeometric optimize(ompl::geometric::PathGeometric path,
                                        const ompl::base::OptimizationObjectivePtr &objective,
                                        const std::shared_ptr<ompl::base::SpaceInformation> &si) {
    ompl::geometric::PathSimplifier simplifier(si);
    simplifier.simplifyMax(path);
    return path;
}

ompl::geometric::PathGeometric optimizeExit(const Apple &apple,
                                            ompl::geometric::PathGeometric path,
                                            const ompl::base::OptimizationObjectivePtr &objective,
                                            const OMPLSphereShellWrapper &shell,
                                            const std::shared_ptr<ompl::base::SpaceInformation> &si) {

    auto shellGoal = std::make_shared<EndEffectorOnShellGoal>(si, shell, apple.center);



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

Apple appleFromApproach(const ompl_interface::ModelBasedStateSpace &state_space,
                        const ompl::geometric::PathGeometric &approach_path) {

    moveit::core::RobotState rs(state_space.getRobotModel());

    state_space.copyToRobotState(rs, approach_path.getState(0));

    rs.update(true);

    return Apple {
            rs.getGlobalLinkTransform("end_effector").translation(),
            {0.0, 0.0, 0.0}
    };
}

std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> optimizeApproachOrder(
        const GreatcircleDistanceHeuristics &gdh,
        const ompl_interface::ModelBasedStateSpace &state_space,
        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches
) {

    auto apples_after_approach = approaches | ranges::views::transform([&](auto pair) {
        return appleFromApproach(state_space, pair.second);
    }) | ranges::to_vector;

    return vectorByOrdering(approaches,
                            ORToolsOrderingStrategy().apple_ordering(apples_after_approach,
                                                                     gdh));
}
