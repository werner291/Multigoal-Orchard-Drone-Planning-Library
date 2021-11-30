#ifndef NEW_PLANNERS_CLUSTERTABLE_H
#define NEW_PLANNERS_CLUSTERTABLE_H


#include <ompl/base/ScopedState.h>
#include "multi_goal_planners.h"

class ClusterTable {

public:
    struct AccessibleTarget {
        size_t target_id;
        ompl::base::ScopedStatePtr at_target;
    };

    struct Cluster {
        ompl::base::ScopedStatePtr representative_state;
        std::vector<AccessibleTarget> accessible_targets;
    };

    std::vector<Cluster> clusters;

    ClusterTable(const GoalSet &goals);

};

/**
 * The cluster-based planner is an attempt to provide a heuristic method to solve the multi-goal planning problem.
 *
 * The planner works as follows:
 *
 * 1. k samples are taken from each goal and pooled into a single set (with a back-reference). This makes for k*n goals.
 * 2. Singleton clusters are built for each goal sample.
 * 3. Using a GNAT, paths are planned from every cluster center to all samples within a threshold Euclidean distance.
 * 4. The clusters are transposed to reveal overlap and local density.
 * 5. The degree of overlap is used to select which clusters to take to the higher level.
 * 6. Repeat from step 3 with threshold multiplied by some constant, until left with a small number of clusters, making around O(log(n)) iterations.
 *
 * Then, this hierarchical structure is used to find a good visitation order of all targets.
 *
 * Finally, a path is planned through these targets with the given visitation order and extra runtime budget.
 */
class ClusterBasedPlanner : public MultiGoalPlanner {
public:
    MultiGoalPlanResult
    plan(GoalSet &goals, const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
         std::chrono::milliseconds time_budget) override;

    std::string getName() override;

    void buildClusters(GoalSet &goals, PointToPointPlanner &point_to_point_planner,
                       const std::vector<StateAtGoal> &goal_samples) const;
};

#endif //NEW_PLANNERS_CLUSTERTABLE_H
