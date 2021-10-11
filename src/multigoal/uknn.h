//
// Created by werner on 30-09-21.
//

#ifndef NEW_PLANNERS_UKNN_H
#define NEW_PLANNERS_UKNN_H

#include "multi_goal_planners.h"

class UnionKNNPlanner : public MultiGoalPlanner {

public:
    UnionKNNPlanner(size_t k,
                    std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection,
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
        os << "U-";
        os << k;
        os << "-NN";
        return os.str();
    }

};


#endif //NEW_PLANNERS_UKNN_H
