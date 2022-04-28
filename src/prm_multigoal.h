//
// Created by werner on 26-4-22.
//

#ifndef NEW_PLANNERS_PRM_MULTIGOAL_H
#define NEW_PLANNERS_PRM_MULTIGOAL_H

#include <moveit/planning_scene/planning_scene.h>
#include "robot_path.h"
#include "procedural_tree_generation.h"

RobotPath planByApples(const moveit::core::RobotState& start_state,
                       const planning_scene::PlanningSceneConstPtr& scene,
                       const std::vector<Apple>& apples);

#endif //NEW_PLANNERS_PRM_MULTIGOAL_H
