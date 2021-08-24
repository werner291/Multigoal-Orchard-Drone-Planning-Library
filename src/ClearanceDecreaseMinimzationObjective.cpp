//
// Created by werner on 8/24/21.
//

#include "build_planning_scene.h"
#include "build_request.h"
#include "EndEffectorConstraintSampler.h"
#include <random_numbers/random_numbers.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/util.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include "ClearanceDecreaseMinimzationObjective.h"

ompl::base::Cost ClearanceDecreaseMinimzationObjective::stateCost(const ompl::base::State *s) const {
    return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
}

ompl::base::Cost ClearanceDecreaseMinimzationObjective::identityCost() const {
    return ompl::base::Cost(0.0);
}

ompl::base::Cost ClearanceDecreaseMinimzationObjective::infiniteCost() const {
    return ompl::base::Cost(std::numeric_limits<double>::infinity());
}

bool ClearanceDecreaseMinimzationObjective::isCostBetterThan(ompl::base::Cost c1, ompl::base::Cost c2) const {
    return c1.value() < c2.value();
}
