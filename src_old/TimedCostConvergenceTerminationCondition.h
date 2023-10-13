
#ifndef NEW_PLANNERS_TIMEDCOSTCONVERGENCETERMINATIONCONDITION_H
#define NEW_PLANNERS_TIMEDCOSTCONVERGENCETERMINATIONCONDITION_H

#include <ompl/base/PlannerTerminationCondition.h>

#include <utility>

/**
 * A PlannerTerminationCondition class that terminates if the solution has not improved for a certain amount of time.
 *
 * Optionally, the termination condition will not trigger if no solutions have yet been providing.
 */
class TimedConvergenceTerminationCondition : public ompl::base::PlannerTerminationCondition {

    /// The cost of the last solution, assumed to decrease over time.
    double last_cost_;

    /// The time of the last solution that improved the result.
    ompl::time::point last_time_;

    /// The amount of time that must pass before the termination condition is met.
    ompl::time::duration time_threshold_;

    /// Whether the termination condition requires at least one solution.
    /// Is set to false when the first solution is found.
    bool await_solution;

    /// Callback for when a new solution is found.
    void processNewSolution(const ompl::base::Cost cost);

    /// Whether the termination condition has been met, also passed as a callback to PlannerTerminationCondition.
    [[nodiscard]] bool should_terminate() const;

public:

    /**
     * Construct a TimedConversionTerminationCondition.
     * @param pdef              The ProblemDefinition; the intermediate solution callback will be overridden.
     * @param timeThreshold     The amount of time that must pass before the termination condition is met.
     * @param awaitSolution     Whether the termination condition requires at least one solution.
     */
    TimedConvergenceTerminationCondition(ompl::base::ProblemDefinition &pdef,
										 ompl::time::duration timeThreshold,
										 bool awaitSolution);

};


#endif //NEW_PLANNERS_TIMEDCOSTCONVERGENCETERMINATIONCONDITION_H
