//
// Created by werner on 30-09-21.
//

#ifndef NEW_PLANNERS_APPROACH_CLUSTERING_H
#define NEW_PLANNERS_APPROACH_CLUSTERING_H

#include "multi_goal_planners.h"

namespace multigoal {

    namespace ob = ompl::base;
    typedef std::shared_ptr<ompl::base::GoalSampleableRegion> GoalRegionPtr;

    typedef std::vector<std::vector<ob::ScopedStatePtr>> GoalApproachTable;

    class ApproachClustering : public MultiGoalPlanner {

        size_t initial_k;
    public:
        ApproachClustering(size_t initialK);

        MultiGoalPlanResult plan(const TreeScene &apples,
                                 const moveit::core::RobotState &start_state,
                                 const robowflex::SceneConstPtr &scene,
                                 const robowflex::RobotConstPtr &robot,
                                 PointToPointPlanner &point_to_point_planner) override;

        std::string getName() override;


        static std::vector<GoalRegionPtr> constructGoalRegions(const TreeScene &apples,
                                                               const ompl::base::SpaceInformationPtr &si);

        static GoalApproachTable
        takeGoalSamples(const ompl::base::SpaceInformationPtr &si, const std::vector<GoalRegionPtr> &goals,
                        int k);

        static void keepBest(const ompl::base::OptimizationObjective &opt, GoalApproachTable &table, int keep_k);
    };

}

#endif //NEW_PLANNERS_APPROACH_CLUSTERING_H
