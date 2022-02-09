#ifndef NEW_PLANNERS_CLUSTERTABLE_H
#define NEW_PLANNERS_CLUSTERTABLE_H

#include <ompl/base/ScopedState.h>
#include <variant>
#include <boost/range/adaptors.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include "multi_goal_planners.h"
#include "../general_utilities.h"
#include "clustering_types.h"
#include "in_cluster_distances.h"
#include "clustering_preselection.h"
#include "clustering_density.h"

namespace clustering {

    std::vector<StateAtGoal> takeInitialSamples(const GoalSet &goals,
                                                const ompl::base::SpaceInformationPtr &si,
                                                int samples_per_goal);

    /**
     * Trivially builds a layer of clusters: one per state at goal.
     */
    std::vector <Cluster> buildTrivialClusters(const std::vector<StateAtGoal> &goal_samples);

    const size_t DEFAULT_CLUSTER_SIZE = 5;

    std::vector<Cluster> create_cluster_candidates(PointToPointPlanner &point_to_point_planner,
                                                   const std::vector<StateAtGoal> &goal_samples,
                                                   const std::vector<Cluster> &clusters,
                                                   const PreselectionStrategy &preselect,
                                                   const PostSelectionStrategy &postselect,
                                                   size_t level,
                                                   double time_per_ptp);

    std::vector<double> inverseDistanceFromCenterDensity(const std::vector<Cluster> &new_clusters);

    /**
     * Taking a vector of clusters as input, this algorithm will select a subset of the available clusters,
     * based on maximum local density or on whether the cluster is an outlier.
     *
     * The algorithm works by building a priority queue of each cluster based on local density, highest density first.
     *
     * It will then extract clusters one-by-one from the queue, adding to the selected set. For each cluster added
     * to the set, the neighbors will be removed from consideration (by lazy deletion), and second-order neighbours
     * will have their local density reduced accordingly (this reduces priority for consideration, which is also
     * implemented via lazy deletion).
     *
     * @param clusters The clusters to pick from.
     * @param densities The local density of each cluster.
     * @return A vector of indices into `clusters`, forming the selected subset.
     */
    std::vector<size_t> select_clusters(const std::vector<Cluster> &clusters, std::vector<double> densities);

    std::vector<std::vector<Cluster>>
    buildClusters(PointToPointPlanner &point_to_point_planner, const std::vector<StateAtGoal> &goal_samples,
                  const PreselectionStrategy &preselect, const PostSelectionStrategy &postselect,
                  const DensityStrategy &densityStrategy);

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
        plan(const GoalSet &goals, const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
             std::chrono::milliseconds time_budget) override;

        std::string getName() override;

    };

}


#endif //NEW_PLANNERS_CLUSTERTABLE_H
