
#include "NewKnnPlanner.h"
#include "DistanceHeuristics.h"
#include <range/v3/all.hpp>
#include <ompl/geometric/PathGeometric.h>

using namespace std;
using namespace ranges;

ompl::NearestNeighborsGNAT<NewKnnPlanner::StateOrGoal>
NewKnnPlanner::buildGNAT(const ompl::base::State *start, const vector <ompl::base::GoalPtr> &goals) const {

    ompl::NearestNeighborsGNAT<StateOrGoal> gnat;

    gnat.setDistanceFunction([&](const StateOrGoal& a, const StateOrGoal& b) -> double {
        if (a.index() == 0) {
            assert(b.index() == 1);
            return distance_heuristics_->state_to_goal(get<const ompl::base::State *>(a), get<GoalIdAndGoal>(b).second);
        }

        if (b.index() == 0) {
            assert(a.index() == 1);
            return distance_heuristics_->state_to_goal(get<const ompl::base::State *>(b), get<GoalIdAndGoal>(a).second);
        }

        return distance_heuristics_->goal_to_goal(get<GoalIdAndGoal>(a).second, get<GoalIdAndGoal>(b).second);
    });

    gnat.add(goals | views::enumerate | views::transform([&](auto&& pair) -> StateOrGoal {
        return {std::make_pair(pair.first, pair.second.get())};
    }) | to_vector);

    return gnat;
}

NewKnnPlanner::PlanResult NewKnnPlanner::plan(
        const ompl::base::SpaceInformationPtr &si,
        const ompl::base::State *start,
        const std::vector <ompl::base::GoalPtr> &goals,
        SingleGoalPlannerMethods& methods) {

    auto gnat = buildGNAT(start, goals);

    std::vector<PathSegment> segments;

    size_t goals_visited = 0;

    ompl::base::State* last_state;

    while (gnat.size() > 0) {

        std::vector<StateOrGoal> nearest_k;

        // Find the k nearest apples.
        gnat.nearestK({last_state}, k, nearest_k);

        std::vector<pair<StateOrGoal,ompl::geometric::PathGeometric>> paths;

        // For all, plan a path
        for (auto &apple: nearest_k) {

            if (auto result = methods.state_to_goal(last_state, goals[get<GoalIdAndGoal>(apple).first])) {
                paths.emplace_back(apple,result.value());
            }
        }

        if (paths.empty()) {
            gnat.remove(nearest_k[0]);
        } else {
            // Pick the shortest path
            auto shortest = paths[0];
            for (auto &path: paths) {
                if (path.second.length() < shortest.second.length()) {
                    shortest = path;
                }
            }
            gnat.remove(shortest.first);
            segments.push_back({get<GoalIdAndGoal>(shortest.first).first, shortest.second});
            last_state = shortest.second.getState(shortest.second.getStateCount() - 1);

            goals_visited += 1;
        }
    }

    return PlanResult { segments };
}

NewKnnPlanner::NewKnnPlanner(const shared_ptr<const OmplDistanceHeuristics> &distanceHeuristics, size_t k) : distance_heuristics_(distanceHeuristics), k(k) {

}

