#include "ATNN.h"
#include "approach_table.h"


MultiGoalPlanResult
multigoal::ATNN::plan(const std::vector<GoalSamplerPtr> &goals, const ompl::base::State *start_state,
                      PointToPointPlanner &point_to_point_planner) {

    // Build a goal approach table.
    GoalApproachTable table = takeGoalSamples(point_to_point_planner.getPlanner()->getSpaceInformation(), goals, 50);

    // Delete any but the five best samples. (TODO: This is rather naive, surely we could vary this, maybe?)
    keepBest(*point_to_point_planner.getOptimizationObjective(), table, 5);

    auto unvisited_nn = buildGNAT(table, point_to_point_planner.getPlanner()->getSpaceInformation());

    MultiGoalPlanResult result;

    // Keep going until the GNAT is empty.
    while (unvisited_nn.size() > 0) {

        const ompl::base::State *segment_start_state = result.segments.empty() ? start_state
                                                                               : result.segments.back().path.getStates().back();

        // Look up k targets closest to it.
        auto nearest = unvisited_nn.nearest({segment_start_state});

        auto ptp_result = point_to_point_planner.planToOmplState(MAX_TIME_PER_TARGET_SECONDS, segment_start_state,
                                                                 nearest.state);

        // If at least one attempt was successful...
        if (ptp_result.has_value()) {

            deleteApproachesFromGNAT(table, unvisited_nn, nearest);

            result.segments.push_back({
                                              nearest.v.value().target_idx,
                                              ptp_result.value()
                                      });

        } else {
            // Just delete the first of nearest neighbour.
            unvisited_nn.remove(nearest); // Better picks here? Maybe delete all?
        }
    }

    return result;
}

void multigoal::ATNN::deleteApproachesFromGNAT(const multigoal::GoalApproachTable &table,
                                               ompl::NearestNeighborsGNAT<multigoal::ATNN::Node> &unvisited_nn,
                                               const Node &nearest) {
    for (size_t approach_idx = 0; approach_idx < table[nearest.v.value().target_idx].size(); approach_idx++) {
        unvisited_nn.remove({Node{
                table[nearest.v.value().target_idx][approach_idx]->get(),
                Visitation{
                        nearest.v.value().target_idx,
                        approach_idx
                }
        }});
    }
}

ompl::NearestNeighborsGNAT<multigoal::ATNN::Node>
multigoal::ATNN::buildGNAT(const multigoal::GoalApproachTable &table, const ompl::base::SpaceInformationPtr &si) {
    // Nearest-Neighbour datastructure for states (with visitation backpointer)
    ompl::NearestNeighborsGNAT<Node> unvisited_nn;
    unvisited_nn.setDistanceFunction([&](const Node &a, const Node &b) {
        return si->distance(a.state, b.state);
    });

    // Add all entries in the GAT to the nearest beighbour structure.
    for (size_t target_idx = 0; target_idx < table.size(); target_idx++) {
        for (size_t approach_idx = 0; approach_idx < table[target_idx].size(); approach_idx++) {
            unvisited_nn.add({Node{
                    table[target_idx][approach_idx]->get(),
                    Visitation{
                            target_idx,
                            approach_idx
                    }
            }});
        }
    }
    return unvisited_nn;
}

std::string multigoal::ATNN::getName() {
    return "ATNN";
}
