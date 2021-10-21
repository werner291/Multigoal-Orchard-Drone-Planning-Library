//
// Created by werner on 30-09-21.
//

#ifndef NEW_PLANNERS_UKNN_H
#define NEW_PLANNERS_UKNN_H

typedef std::function<Eigen::Vector3d(const ompl::base::Goal *)> GoalProjectionFn;

typedef std::function<Eigen::Vector3d(const ompl::base::State *)> StateProjectionFn;

#include "multi_goal_planners.h"

class UnionKNNPlanner : public MultiGoalPlanner {

    size_t k;
    GoalProjectionFn goalProjection_;
    StateProjectionFn stateProjection_;

public:
    UnionKNNPlanner(size_t k,
                    GoalProjectionFn goalProjection,
                    StateProjectionFn stateProjection);

    MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals,
                             const ompl::base::State *start_state,
                             PointToPointPlanner &point_to_point_planner) override;

    std::string getName() override {
        std::ostringstream os;
        os << "U-";
        os << k;
        os << "-NN";
        return os.str();
    }

};


#endif //NEW_PLANNERS_UKNN_H
