//
// Created by werner on 8/24/21.
//

#ifndef NEW_PLANNERS_CLEARANCEDECREASEMINIMIZATIONOBJECTIVE_H
#define NEW_PLANNERS_CLEARANCEDECREASEMINIMIZATIONOBJECTIVE_H

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>

/**
 * An optimization objective inspired by TRRT and BiTRRT that assigns
 * costs to increasing the inverse of the clearance.
 *
 * Effectively, a planner will be "punished" for decreasing the clearance,
 * but is not "rewarded" for increasing the clearance or taking a shorter
 * path to some extent.
 *
 * The intuition is that the planner will then produce "naturally good" paths,
 * that avoid costly expensive passages and prefer safe, long-ways-around,
 * while avoiding the noise-sensitivity of minimax algorithms.
 *
 * For instance, the planner will be rewarded for moving around a dangerous area without
 * approaching it, rather than attempting to pass through a thin section of it.
 */
class ClearanceDecreaseMinimizationObjective : public ompl::base::MechanicalWorkOptimizationObjective {

    /**
     * \brief State cost is the inverse of the clearance.
     *
     * Inverse was picked, rather than clearance itself, since the MechanicalWorkOptimizationObjective
     * assumes that higher costs are worse.
     */
    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

    /**
     * \brief Identity cost is zero; that's infinite clearance.
     */
    [[nodiscard]] ompl::base::Cost identityCost() const override;

    /**
     * \brief Infinite cost is just infinty.
     */
    [[nodiscard]] ompl::base::Cost infiniteCost() const override;

    /**
     * This just compares the cost values, since higher cost is worse.
     */
    [[nodiscard]] bool isCostBetterThan(ompl::base::Cost c1, ompl::base::Cost c2) const override;

public:
    explicit ClearanceDecreaseMinimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                    double pathLengthWeight = 0.00001)
            : MechanicalWorkOptimizationObjective(si, pathLengthWeight) {
        description_ = "Minimize reduction of clearance";
    }

};

#endif //NEW_PLANNERS_CLEARANCEDECREASEMINIMIZATIONOBJECTIVE_H
