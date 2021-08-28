
#ifndef NEW_PLANNERS_INVERSECLEARANCEINTEGRALOBJECTIVE_H
#define NEW_PLANNERS_INVERSECLEARANCEINTEGRALOBJECTIVE_H

#include <ompl/base/objectives/StateCostIntegralObjective.h>

class InverseClearanceIntegralObjective : public ompl::base::StateCostIntegralObjective {


public:
    InverseClearanceIntegralObjective(const ompl::base::SpaceInformationPtr &si,
                                      bool enableMotionCostInterpolation)
            : StateCostIntegralObjective(si, enableMotionCostInterpolation) {}

private:
    ompl::base::Cost stateCost(const ompl::base::State *s) const override {
        return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
    }

};


#endif //NEW_PLANNERS_INVERSECLEARANCEINTEGRALOBJECTIVE_H
