#include "TimedCostConvergenceTerminationCondition.h"

TimedConvergenceTerminationCondition::TimedConvergenceTerminationCondition(ompl::base::ProblemDefinition &pdef,
																		   ompl::time::duration timeThreshold,
																		   bool awaitSolution)
        : PlannerTerminationCondition([this]() { return this->should_terminate(); }),
          last_cost_(INFINITY),
          last_time_(ompl::time::now()), time_threshold_(timeThreshold), await_solution(awaitSolution) {

    auto callback = [this](const ompl::base::Planner * /*planner*/,
                           const std::vector<const ompl::base::State *> & /*states*/,
                           const ompl::base::Cost cost) {
        this->processNewSolution(cost);
    };

    pdef.setIntermediateSolutionCallback(callback);

}

void TimedConvergenceTerminationCondition::processNewSolution(const ompl::base::Cost cost) {
    if (cost.value() < last_cost_) {
        last_cost_ = cost.value();
        last_time_ = ompl::time::now();

        // We have a solution, stop waiting for a solution.
        await_solution = false;
    }
}

bool TimedConvergenceTerminationCondition::should_terminate() const {
    return !await_solution && ompl::time::now() - this->last_time_ > this->time_threshold_;
}
