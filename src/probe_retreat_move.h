
#ifndef NEW_PLANNERS_PROBE_RETREAT_MOVE_H
#define NEW_PLANNERS_PROBE_RETREAT_MOVE_H

#include <Eigen/Core>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <geometric_shapes/shapes.h>
#include "procedural_tree_generation.h"
#include "multigoal/PointToPointPlanner.h"
#include "Highway.h"



ompl::geometric::PathGeometric
plan_probe_retreat_slide(const std::vector<Apple> &apples_in_order, const ompl::base::State *initial_state,
                         const ompl::base::SpaceInformationPtr &si,
                         const std::function<void(const Apple &apple, ompl::base::State *)> &state_outside_tree,
                         const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State *,
                                                                                           ompl::base::State *)> &plan_state_to_state,
                         const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State *,
                                                                                           const Apple &apple)> &plan_state_to_apple,
                         const Highway<Apple>& highway,
                         bool simplify);

#endif //NEW_PLANNERS_PROBE_RETREAT_MOVE_H
