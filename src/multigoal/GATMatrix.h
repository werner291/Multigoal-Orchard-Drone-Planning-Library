
#ifndef NEW_PLANNERS_GATMATRIX_H
#define NEW_PLANNERS_GATMATRIX_H

#include "multi_goal_planners.h"
#include "approach_table.h"

namespace multigoal {

    class GATDistanceMatrix {

        std::vector< // By target ID
                std::vector< // By approach ID
                        double> > distances_from_start;

        std::vector< // By target ID
                std::vector< // By second target ID
                        std::vector< // By first approach ID
                                std::vector<double> // By second approach ID
                        >
                >
        > distances;

        double lookup_start(const Visitation &a);

        double lookup(const Visitation &a, const Visitation &b);

        static GATDistanceMatrix
        fullMatrix(const ompl::base::State *start_state, const GoalApproachTable &gat, PointToPointPlanner &ptp);

    };

}

#endif //NEW_PLANNERS_GATMATRIX_H
