#ifndef NEW_PLANNERS_ATNN_H
#define NEW_PLANNERS_ATNN_H

#include "multi_goal_planners.h"
#include "approach_table.h"

namespace multigoal {
    class ATNN : public MultiGoalPlanner {

        /// Just a pointer to an OMPL State with a back-pointer into a GoalApproachTable.
        struct Node {
            const ompl::base::State *state{};
            std::optional<multigoal::Visitation> v; // Optional because nearest-neighbour lookup needs a query value where the back-pointer is nonsensical.

            bool operator==(const Node &other) const {
                return state == other.state; /// Yes, we ignore the visitation on purpose.
            }

            bool operator!=(const Node &other) const {
                return state != other.state;
            }
        };

        static ompl::NearestNeighborsGNAT<ATNN::Node>
        buildGNAT(const multigoal::GoalApproachTable &table, const ompl::base::SpaceInformationPtr &si);

        static void deleteApproachesFromGNAT(const multigoal::GoalApproachTable &table,
                                             ompl::NearestNeighborsGNAT<Node> &unvisited_nn, const ATNN::Node &nearest);

    public:
        MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals, const ompl::base::State *start_state,
                                 PointToPointPlanner &point_to_point_planner) override;

        std::string getName() override;

    };
}

#endif //NEW_PLANNERS_ATNN_H
