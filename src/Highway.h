//
// Created by werner on 15-03-22.
//

#ifndef NEW_PLANNERS_HIGHWAY_H
#define NEW_PLANNERS_HIGHWAY_H

#include <ompl/geometric/PathGeometric.h>
#include "procedural_tree_generation.h"

/**
 * Abstractly represents a region of the planning space through which
 * planning is very cheap, and can be done very quickly, using some
 * pre-knowledge of the space.
 *
 * Places on or near the highway are intentionally kept abstract to avoid forcing the user
 * to explicitly generate valid states in the state space.
 *
 * @tparam P The type that designates locations on or near the highway, usually a projection of the state space.
 */
template<typename P>
class Highway {

public:

    /**
     * Given an abstract location designation, generates an OMPL state that is "on the highway",
     * from which a path can be computed very quickly to another part of the space.
     *
     * This state is guaranteed to be the same as the start/end point of the paths planned by Highway::fast_plan.
     *
     * @param a The location designation.
     * @param result The state tho be written to.
     */
    virtual void on_ramp(const P& a, ompl::base::State* result) const = 0;

    /**
     * Fast-plan a path through the highway between the on-ramps corresponding to the location designators a and b.
     */
    virtual ompl::geometric::PathGeometric fast_plan(const P& a, const P& b) const = 0;

    virtual double heuristic_distance(const P&a, const P& b) const = 0;

    /**
     * Take an OMPL state, and project it into the location designation space.
     */
    virtual P project_state(ompl::base::State* near_state) const = 0;

};




#endif //NEW_PLANNERS_HIGHWAY_H
