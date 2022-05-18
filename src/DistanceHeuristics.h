
#ifndef NEW_PLANNERS_DISTANCEHEURISTICS_H
#define NEW_PLANNERS_DISTANCEHEURISTICS_H

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include "ompl_custom.h"
#include "greatcircle.h"

class OmplDistanceHeuristics {

public:
    virtual double state_to_goal(const ompl::base::State *, const ompl::base::Goal *) const = 0;

    virtual double goal_to_goal(const ompl::base::Goal *, const ompl::base::Goal *) const = 0;

    [[nodiscard]] virtual std::string name() const = 0;
};

class EuclideanOmplDistanceHeuristics : public OmplDistanceHeuristics {

    std::shared_ptr<DroneStateSpace> state_space_;

public:
    EuclideanOmplDistanceHeuristics(std::shared_ptr<DroneStateSpace> stateSpace);

    double state_to_goal(const ompl::base::State *a, const ompl::base::Goal *b) const override;

    double goal_to_goal(const ompl::base::Goal *a, const ompl::base::Goal *b) const override;

    [[nodiscard]] std::string name() const override;

};

class GreatCircleOmplDistanceHeuristics : public OmplDistanceHeuristics {

    GreatCircleMetric gcm;
    std::shared_ptr<DroneStateSpace> state_space_;

public:
    GreatCircleOmplDistanceHeuristics(GreatCircleMetric gcm, std::shared_ptr<DroneStateSpace> stateSpace);

    double state_to_goal(const ompl::base::State *a, const ompl::base::Goal *b) const override;

    double goal_to_goal(const ompl::base::Goal *a, const ompl::base::Goal *b) const override;

    [[nodiscard]] std::string name() const override;

};

#endif //NEW_PLANNERS_DISTANCEHEURISTICS_H
