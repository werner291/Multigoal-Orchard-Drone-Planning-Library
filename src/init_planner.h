//
// Created by werner on 8/25/21.
//

#ifndef NEW_PLANNERS_INIT_PLANNER_H
#define NEW_PLANNERS_INIT_PLANNER_H

#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_ompl/ompl_interface.h>

std::shared_ptr<robowflex::OMPL::OMPLInterfacePlanner> init_planner(std::shared_ptr<robowflex::Robot> drone,
                                                                    std::shared_ptr<robowflex::Scene> scene);

#endif //NEW_PLANNERS_INIT_PLANNER_H
