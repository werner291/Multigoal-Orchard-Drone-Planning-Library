
#ifndef MULTI_GOAL_PLANNERS_H
#define MULTI_GOAL_PLANNERS_H

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <json/value.h>
#include "../ompl_custom.h"
#include "PointToPointPlanner.h"

// For optimizing planners, approximately how much time should be left to the planning operation per target, all other things equal.
const double MAX_TIME_PER_TARGET_SECONDS = 0.1;

typedef std::shared_ptr<ompl::base::GoalSampleableRegion> GoalSamplerPtr;
typedef std::function<Eigen::Vector3d(const GoalSamplerPtr &)> GoalProjectionFn;
typedef std::function<Eigen::Vector3d(const ompl::base::State *)> StateProjectionFn;
typedef const std::vector<GoalSamplerPtr> GoalSet;

struct PointToPointPath {
    size_t to_goal;
    ompl::geometric::PathGeometric path;
};

/**
 * Result struct of a multi-goal planning operation.
 */
struct MultiGoalPlanResult {

    struct ReplacementSpec {
        size_t from;
        size_t until;
        std::vector<size_t> target_ids;
    };

    std::vector<PointToPointPath> segments;

    /**
     * \brief Computes which parts of a MultiGoalPlanResult should be replaced in order to realize a swap
     * of targets i and j (in the current MultiGoalPlanResult's order). Each range is strictly disjoint.
     *
     * Requires that i < j, and that i,j are valid indices in the ATSolution.
     */
    std::vector<ReplacementSpec> replacements_for_swap(size_t num_goals, size_t i, size_t j);

    const ompl::base::State *state_after_segments(size_t count, const ompl::base::State *start_state) const;

    /// Check internal invariants via assertions (crashes if violated)
    void check_valid(const GoalSet &table, const ompl::base::SpaceInformation &si) const;

    /// Apply a set of replacements.
    void apply_replacements(const std::vector<ReplacementSpec> &replacement_specs,
                            const std::vector<PointToPointPath> &computed_replacements);

    double total_length() {
        return std::accumulate(segments.begin(), segments.end(), 0.0, [](double &a, const PointToPointPath &b) {
            return a + b.path.length();
        });
    }

    [[nodiscard]] double originalCost(const std::vector<ReplacementSpec> &replacement_specs) const;

    [[nodiscard]] double newCost(const std::vector<PointToPointPath> &computed_replacements) const;
};


/**
 * A meta-planner that, given a number of targets to visit,
 * attempts to plan a trajectory that brings the end-effector
 * of the robot close to each target.
 */
class MultiGoalPlanner {

public:

    /**
     * Plan the trajectory.
     *
     * @param goals                  List of GoalSampleableRegion, the planner will attempt to visit all.
     * @param start_state            The state of the robot at the start.
     * @param point_to_point_planner Wrapper for an OMPL planner and an optimization objective
     */
    virtual MultiGoalPlanResult plan(GoalSet &goals,
                                     const ompl::base::State *start_state,
                                     PointToPointPlanner &point_to_point_planner,
                                     std::chrono::milliseconds time_budget) = 0;

    /**
     * \brief Returns the name of the meta-planner. Does NOT include any sub-planners or optimization objectives.
     */
    virtual std::string getName() = 0;

    /// Computes a vector of `PathGeometric` corresponding, in order, to the `replacements`.
    /// Computation is fallible, and the `optional` will be filled only on success.
    static std::optional<std::vector<PointToPointPath>>
    computeNewPathSegments(const ompl::base::State *start_state,
                           PointToPointPlanner &point_to_point_planner, const GoalSet &goals,
                           const MultiGoalPlanResult &solution,
                           const std::vector<MultiGoalPlanResult::ReplacementSpec> &replacements,
                           double maxTimePerSegment);
};


#endif //MULTI_GOAL_PLANNERS_H
