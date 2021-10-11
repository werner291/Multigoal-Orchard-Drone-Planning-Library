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
struct GNATNode {
    size_t goal{};
    Eigen::Vector3d goal_pos;

    bool operator!=(const GNATNode &other);
};

class KNNPlanner : public MultiGoalPlanner {

public:
    explicit KNNPlanner(size_t k, std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection,
                        std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection);

private:
    size_t k;
    std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection_;
    std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection_;

public:
    MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals,
                             const ompl::base::State *start_state,
                             PointToPointPlanner &point_to_point_planner) override;

    std::string getName() override {
        std::ostringstream os;
        os << k;
        os << "-NN";
        return os.str();
    }

};

#endif //NEW_PLANNERS_KNN_H
