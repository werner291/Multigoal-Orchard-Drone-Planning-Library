//
// Created by werner on 30-09-21.
//

#ifndef NEW_PLANNERS_UKNN_H
#define NEW_PLANNERS_UKNN_H

#include "multi_goal_planners.h"

class UnionKNNPlanner : public MultiGoalPlanner {

public:
    explicit UnionKNNPlanner(size_t k);

private:
    size_t k;

public:
    MultiGoalPlanResult plan(const TreeScene &apples,
                             const moveit::core::RobotState &start_state,
                             const robowflex::SceneConstPtr &scene,
                             const robowflex::RobotConstPtr &robot,
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
