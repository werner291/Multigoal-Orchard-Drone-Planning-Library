//
// Created by werner on 18-08-21.
//

#ifndef NEW_PLANNERS_BUILD_PLANNING_SCENE_H
#define NEW_PLANNERS_BUILD_PLANNING_SCENE_H

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include "procedural_tree_generation.h"

/**
 * Create a PlanningScene message that represents a scene with a tree in it.
 * @return The PlanningScene message.
 */
PlanningScene establishPlanningScene(int branchingDepth, int numberOfApples);

void buildPlanningScene(std::shared_ptr<robowflex::Robot> &drone, std::shared_ptr<robowflex::Scene> &scene,
                        PlanningScene &tree_scene);

#endif //NEW_PLANNERS_BUILD_PLANNING_SCENE_H
