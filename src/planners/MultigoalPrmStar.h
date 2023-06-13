//
// Created by werner on 26-4-22.
//

#ifndef NEW_PLANNERS_MULTIGOALPRMSTAR_H
#define NEW_PLANNERS_MULTIGOALPRMSTAR_H

#include <moveit/planning_scene/planning_scene.h>
#include "../RobotPath.h"
#include "../utilities/experiment_utils.h"
#include "../procedural_tree_generation.h"
#include "MultiGoalPlanner.h"
#include <ompl/geometric/planners/prm/PRMstar.h>

class PRMCustom : public ompl::geometric::PRMstar {

public:
    explicit PRMCustom(const ompl::base::SpaceInformationPtr &si);

    std::vector<Vertex> tryConnectGoal(ompl::base::GoalSampleableRegion &goal_region, size_t max_samples);

    Vertex insert_state(const ompl::base::State *st);

    /**
     * Try to connect two vertices/states with a path. As an added optimization,
     * first check if the two vertices are even in the same component.
     *
     * @param u Start vertex
     * @param v End vertex
     * @return A path if one exists, or nullptr otherwise
     */
    ompl::base::PathPtr path_distance(Vertex start, Vertex goal);

    bool same_component(Vertex v, Vertex u);

};

MultiApplePlanResult
planByApples(const moveit::core::RobotState &start_state, const planning_scene::PlanningSceneConstPtr &scene,
             const std::vector<Apple> &apples, double prm_build_time, bool optimize_segments, size_t samplesPerGoal);

class MultigoalPrmStar : public MultiGoalPlanner {

    double prm_build_time;
    size_t samplesPerGoal;
    bool optimize_segments;
	double aabb_padding = 1.0;
public:
    MultigoalPrmStar(double prmBuildTime, size_t samplesPerGoal, bool optimizeSegments, double aabb_padding = 1.0);

public:
    PlanResult plan(const ompl::base::SpaceInformationPtr &si,
					const ompl::base::State *start,
					const std::vector<ompl::base::GoalPtr> &goals,
					const AppleTreePlanningScene &planning_scene,
					ompl::base::PlannerTerminationCondition &ptc) override;

    [[nodiscard]] Json::Value parameters() const override;

    [[nodiscard]] std::string name() const override;


};

#endif //NEW_PLANNERS_MULTIGOALPRMSTAR_H
