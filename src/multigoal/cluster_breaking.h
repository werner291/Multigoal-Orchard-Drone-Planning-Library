//
// Created by werner on 30-09-21.
//

#ifndef NEW_PLANNERS_CLUSTER_BREAKING_H
#define NEW_PLANNERS_CLUSTER_BREAKING_H

#include "multi_goal_planners.h"

class ApproachClustering : public MultiGoalPlanner {

    size_t initial_k;
public:
    ApproachClustering(size_t initialK);

public:
    MultiGoalPlanResult plan(const TreeScene &apples,
                             const moveit::core::RobotState &start_state,
                             const robowflex::SceneConstPtr &scene,
                             const robowflex::RobotConstPtr &robot,
                             PointToPointPlanner &point_to_point_planner) override;

    std::string getName() override;

};

#endif //NEW_PLANNERS_CLUSTER_BREAKING_H
