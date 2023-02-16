//
// Created by werner on 16-2-23.
//

#ifndef NEW_PLANNERS_VISUALIZE_DYNAMIC_H
#define NEW_PLANNERS_VISUALIZE_DYNAMIC_H

#include "../TreeMeshes.h"
#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../DynamicGoalVisitationEvaluation.h"

int visualizeEvaluation(const TreeMeshes &meshes,
						const AppleTreePlanningScene &scene,
						const moveit::core::RobotModelPtr &robot,
						const moveit::core::RobotState &start_state,
						const std::vector<AppleDiscoverabilityType> &apple_discoverability,
						DynamicGoalVisitationEvaluation &eval);

#endif //NEW_PLANNERS_VISUALIZE_DYNAMIC_H
