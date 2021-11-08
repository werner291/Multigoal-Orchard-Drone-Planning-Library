//
// Created by werner on 20-10-21.
//

#include "RandomizedTwoOpt.h"

#include <utility>

MultiGoalPlanResult RandomizedTwoOpt::plan(const std::vector<GoalSamplerPtr> &goals,
                                           const ompl::base::State *start_state,
                                           PointToPointPlanner &point_to_point_planner,
                                           std::chrono::milliseconds time_budget) {

    auto deadline = std::chrono::steady_clock::now() + time_budget;

    std::chrono::milliseconds nn_time((long) ((double) time_budget.count() * nnBudgetFraction));
    auto solution = initialAttemptPlanner_->plan(goals, start_state, point_to_point_planner, nn_time);

    if (solution.segments.size() < 2) {
        std::cerr << "Solution from nearest-neighbor algorithm too short." << std::endl;
        return solution;
    }

    while (std::chrono::steady_clock::now() < deadline) {

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

        size_t i = std::uniform_int_distribution<size_t>(0, solution.segments.size() - 1)(gen);
        size_t j = std::uniform_int_distribution<size_t>(0, solution.segments.size() - 2)(gen);
        if (j >= i) j += 1; // This *should* produce uniformly random numbers without replacement.
        else std::swap(i, j);

        assert(i < j && j < solution.segments.size());

        trySwap(goals, solution, point_to_point_planner, i, j, start_state, (double) ptp_budget_.count() / 1000.0);

    }

    return solution;
}

std::string RandomizedTwoOpt::getName() {

    std::stringstream ss;
    ss << initialAttemptPlanner_->getName() << "*" << nnBudgetFraction;
    ss << "+" "Prob2OPT";
    if (useCostRejectionHeuristic) ss << "-H";
    if (keepGoingOnImprovement) ss << "-K";
    ss << "-ptp" << ptp_budget_.count() << "ms";

    return ss.str();
}

void RandomizedTwoOpt::trySwap(const std::vector<GoalSamplerPtr> &goals, MultiGoalPlanResult &result,
                               PointToPointPlanner &planner, size_t i, size_t j, const ompl::base::State *start_state,
                               double maxTimePerSegment) {

    std::cout << "Attempted swapping " << i << ", " << j << std::endl;

    auto replacement_spec = result.replacements_for_swap(goals.size(), i, j);

    const double originalCost = result.originalCost(replacement_spec);

    if (useCostRejectionHeuristic) {

        double heuristicOldCost = 0.0;
        for (const auto &item: replacement_spec)
            for (int i = item.from; i < item.until; ++i) {
                if (i == item.from)
                    heuristicOldCost += this->betweenStateAndGoalHeuristic_(result.state_after_segments(i, start_state),
                                                                            goals[result.segments[i].to_goal].get());
                else
                    heuristicOldCost += this->betweenGoalDistanceHeuristic_(goals[result.segments[i - 1].to_goal].get(),
                                                                            goals[result.segments[i].to_goal].get());
            }

        const double heuristicNewCost = this->computeNewPathLengthLowerbound(start_state, planner, goals, result,
                                                                             replacement_spec);

        if (heuristicNewCost > heuristicOldCost * 1.5 /* TODO determine parameter value, or randomize */) {
            std::cout << "Rejected by heuristic." << std::endl;
            return;
        }
    }

    auto computed = computeNewPathSegments(
            start_state,
            planner,
            goals,
            result,
            replacement_spec, maxTimePerSegment);

    if (computed) {

        double newCost = result.newCost(*computed);

        if (newCost < originalCost) {
            double before = result.total_length();
            // FIXME Try copying, applying, and THEN checking lengths, just in case the comparison method is broken.
            result.apply_replacements(replacement_spec, *computed);
            double after = result.total_length();

            std::cout << "Improved from " << before << " to " << after << std::endl;
            assert(after < before);
            result.check_valid(goals, *planner.getPlanner()->getSpaceInformation());

            if (keepGoingOnImprovement) {
                auto computed2 = computeNewPathSegments(
                        start_state,
                        planner,
                        goals,
                        result,
                        replacement_spec, maxTimePerSegment);

                double secondNewCost = result.newCost(*computed2);

                if (secondNewCost < newCost) {
                    double before2 = result.total_length();
                    result.apply_replacements(replacement_spec, *computed);
                    double after2 = result.total_length();
                    std::cout << "Kept going: improved from " << before << " to " << after << std::endl;
                } else {
                    std::cout << "Further improvement failed." << after << std::endl;
                }
            }

        }
    }

}

double RandomizedTwoOpt::computeNewPathLengthLowerbound(const ompl::base::State *start_state,
                                                        PointToPointPlanner &point_to_point_planner,
                                                        const GoalSet &goals,
                                                        const MultiGoalPlanResult &solution,
                                                        const std::vector<MultiGoalPlanResult::ReplacementSpec> &replacements) {

    double total = 0.0;

    for (const auto &repl: replacements) {
        // Instead of invalidating every subsequent solution segment, we instead lock the final state of the to-be-replaced segments
        // to be the starting state of the next segment, if any.
        std::optional<const ompl::base::State *> to_state;
        if (repl.until < solution.segments.size()) to_state = solution.state_after_segments(repl.until, start_state);

        assert(repl.until == solution.segments.size() || goals[repl.target_ids.back()]->isSatisfied(*to_state));

        std::variant<const ompl::base::State *, const ompl::base::Goal *> heuristicPlanFrom = solution.state_after_segments(
                repl.from, start_state);

        for (unsigned long target_id: repl.target_ids) {

            if (std::holds_alternative<const ompl::base::State *>(heuristicPlanFrom)) {
                // For better accuracy, we'd use the final state. This provides a good lower bound, though (hopefully)
                total += betweenStateAndGoalHeuristic_(
                        std::get<const ompl::base::State *>(heuristicPlanFrom),
                        goals[target_id].get()
                );
            } else {
                total += betweenGoalDistanceHeuristic_(
                        std::get<const ompl::base::Goal *>(heuristicPlanFrom),
                        goals[target_id].get()
                );
            }

            heuristicPlanFrom = goals[target_id].get();

        }
    }

    return total;

}


RandomizedTwoOpt::RandomizedTwoOpt(std::shared_ptr<MultiGoalPlanner> initialAttemptPlanner,
                                   std::function<double(const ompl::base::Goal *,
                                                        const ompl::base::Goal *)> betweenGoalDistanceHeuristic,
                                   std::function<double(const ompl::base::State *,
                                                        const ompl::base::Goal *)> betweenStateAndGoalHeuristic,
                                   bool useCostRejectionHeuristic, bool keepGoingOnImprovement,
                                   double nnBudgetFraction,
                                   std::chrono::milliseconds ptp_budget) :
        initialAttemptPlanner_(std::move(initialAttemptPlanner)),
        betweenGoalDistanceHeuristic_(std::move(betweenGoalDistanceHeuristic)),
        betweenStateAndGoalHeuristic_(std::move(betweenStateAndGoalHeuristic)),
        useCostRejectionHeuristic(useCostRejectionHeuristic), keepGoingOnImprovement(keepGoingOnImprovement),
        nnBudgetFraction(nnBudgetFraction), ptp_budget_(ptp_budget) {}
