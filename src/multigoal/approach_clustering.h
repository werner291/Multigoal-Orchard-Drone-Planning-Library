//
// Created by werner on 30-09-21.
//

#ifndef NEW_PLANNERS_APPROACH_CLUSTERING_H
#define NEW_PLANNERS_APPROACH_CLUSTERING_H

#include "multi_goal_planners.h"
#include "approach_table.h"

namespace multigoal {

    namespace ob = ompl::base;
    typedef std::shared_ptr<ompl::base::GoalSampleableRegion> GoalRegionPtr;

    class ApproachClustering : public MultiGoalPlanner {

        size_t initial_k;
    public:
        ApproachClustering(size_t initialK);

        MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals,
                                 const ompl::base::State *start_state,
                                 PointToPointPlanner &point_to_point_planner) override;

        std::string getName() override;

        static std::vector<GoalRegionPtr> constructGoalRegions(const TreeScene &apples,
                                                               const ompl::base::SpaceInformationPtr &si);

    };

}

#endif //NEW_PLANNERS_APPROACH_CLUSTERING_H
