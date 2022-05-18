//
// Created by werner on 26-4-22.
//

#ifndef NEW_PLANNERS_PRM_MULTIGOAL_H
#define NEW_PLANNERS_PRM_MULTIGOAL_H

#include <moveit/planning_scene/planning_scene.h>
#include "robot_path.h"
#include "experiment_utils.h"
#include "procedural_tree_generation.h"
#include "NewMultiGoalPlanner.h"

MultiApplePlanResult
planByApples(const moveit::core::RobotState &start_state, const planning_scene::PlanningSceneConstPtr &scene,
             const std::vector<Apple> &apples, double prm_build_time, bool optimize_segments, size_t samplesPerGoal);

class MultigoalPrmStar : public NewMultiGoalPlanner {

    double prm_build_time;
    size_t samplesPerGoal;
    bool optimize_segments;
public:
    MultigoalPrmStar(double prmBuildTime, size_t samplesPerGoal, bool optimizeSegments);

public:
    PlanResult plan(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                    const std:: vector<ompl::base::GoalPtr> &goals,
                    SingleGoalPlannerMethods &methods) override;

};

#endif //NEW_PLANNERS_PRM_MULTIGOAL_H
