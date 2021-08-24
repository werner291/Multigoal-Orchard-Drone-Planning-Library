//
// Created by werner on 8/24/21.
//

#ifndef NEW_PLANNERS_CLEARANCEDECREASEMINIMZATIONOBJECTIVE_H
#define NEW_PLANNERS_CLEARANCEDECREASEMINIMZATIONOBJECTIVE_H

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>

class ClearanceDecreaseMinimzationObjective : public ompl::base::MechanicalWorkOptimizationObjective {

    /** \brief Returns a cost with a value of 1. */
    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

    [[nodiscard]] ompl::base::Cost identityCost() const override;

    [[nodiscard]] ompl::base::Cost infiniteCost() const override;

    [[nodiscard]] bool isCostBetterThan(ompl::base::Cost c1, ompl::base::Cost c2) const override;

public:
    explicit ClearanceDecreaseMinimzationObjective(const ompl::base::SpaceInformationPtr &si, double pathLengthWeight = 0.00001)
            : MechanicalWorkOptimizationObjective(si, pathLengthWeight) {
        description_ = "Minimize reduction of clearance";
    }

};

#endif //NEW_PLANNERS_CLEARANCEDECREASEMINIMZATIONOBJECTIVE_H
