#include "ClearanceDecreaseMinimizationObjective.h"

ompl::base::Cost ClearanceDecreaseMinimizationObjective::stateCost(const ompl::base::State *s) const {
    return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
}

ompl::base::Cost ClearanceDecreaseMinimizationObjective::identityCost() const {
    return ompl::base::Cost(0.0);
}

ompl::base::Cost ClearanceDecreaseMinimizationObjective::infiniteCost() const {
    return ompl::base::Cost(std::numeric_limits<double>::infinity());
}

bool ClearanceDecreaseMinimizationObjective::isCostBetterThan(ompl::base::Cost c1, ompl::base::Cost c2) const {
    return c1.value() < c2.value();
}
