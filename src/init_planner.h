//
// Created by werner on 8/25/21.
//

#ifndef NEW_PLANNERS_INIT_PLANNER_H
#define NEW_PLANNERS_INIT_PLANNER_H

#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_ompl/ompl_interface.h>
#include <ompl/geometric/SimpleSetup.h>


typedef const std::function<std::shared_ptr<ompl::base::OptimizationObjective>(
        const ompl::geometric::SimpleSetupPtr &)> ObjectiveFactory;

std::shared_ptr<robowflex::OMPL::OMPLInterfacePlanner>
init_planner(const std::shared_ptr<robowflex::Robot> &drone,
             const std::shared_ptr<robowflex::Scene> &scene,
             ObjectiveFactory &allocateOptimizationObjective);

#endif //NEW_PLANNERS_INIT_PLANNER_H
