
#ifndef NEW_PLANNERS_PROBE_RETREAT_MOVE_H
#define NEW_PLANNERS_PROBE_RETREAT_MOVE_H

#include "SphereShell.h"

ompl::geometric::PathGeometric retreat_travel_probe(
        std::shared_ptr<ompl::base::SpaceInformation> &si,
        OMPLSphereShellWrapper &shell,
        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
        size_t retreat_idx,
        size_t approach_idx,
        bool simplify);

std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> planApproaches(
        const std::vector<Apple> &apples_in_order,
        const ompl::base::OptimizationObjectivePtr &objective,
        OMPLSphereShellWrapper &shell,
        const std::shared_ptr<ompl::base::SpaceInformation> &si);

void optimizeExit(const Apple &apple, ompl::geometric::PathGeometric& path,
                  const ompl::base::OptimizationObjectivePtr &objective,
                  OMPLSphereShellWrapper &shell,
                  const std::shared_ptr<ompl::base::SpaceInformation> &si);

ompl::geometric::PathGeometric planFullPath(
        std::shared_ptr<ompl::base::SpaceInformation> &si,
        ompl::base::State* start,
        OMPLSphereShellWrapper &shell,
        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches);

#endif //NEW_PLANNERS_PROBE_RETREAT_MOVE_H
