//
// Created by werner on 30-09-21.
//

#ifndef NEW_PLANNERS_KNN_H
#define NEW_PLANNERS_KNN_H

#include "multi_goal_planners.h"

/**
 * K-NN planner, whereby the apples are placed into a GNAT.
 *
 * The planner runs on a loop; every iteration, the k apples closest to the end-effector are chosen as candidates.
 * The point-to-point planner is run for each, with a maximum time of (MAX_TIME_PER_TARGET_SECONDS/k) to compensate
 * for the fact that it is run k times, with the shortest path picked as the best. The target closest to the end-effector
 * after that path is removed from the GNAT. If all ptp planner attempts fail, the target closest to the end-effector is
 * deleted from the GNAT and will not be visited.
 *
 * The planner terminates when the GNAT is empty.
 */
class KNNPlanner : public MultiGoalPlanner {

public:
    explicit KNNPlanner(size_t k, GoalProjectionFn goalProjection,
                        std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection,
                        double budgetBiasFactor);

private:
    size_t k;
    GoalProjectionFn goalProjection_;
    StateProjectionFn stateProjection_;

    double budgetBiasFactor = 1.0;

public:
    MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals,
                             const ompl::base::State *start_state,
                             PointToPointPlanner &point_to_point_planner,
                             std::chrono::milliseconds time_budget) override;

    std::string getName() override {
        std::ostringstream os;
        os << k;
        os << "-NN";
        if (budgetBiasFactor != 1.0) { os << ":" << budgetBiasFactor; }
        return os.str();
    }

};

#endif //NEW_PLANNERS_KNN_H
