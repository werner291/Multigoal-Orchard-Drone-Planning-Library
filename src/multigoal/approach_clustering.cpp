//
// Created by werner on 30-09-21.
//

#include "approach_clustering.h"
#include <ompl/base/goals/GoalState.h>
#include <random>

using namespace multigoal;


MultiGoalPlanResult
ApproachClustering::plan(const std::vector<GoalSamplerPtr> &goals,
                         const ompl::base::State *start_state,
                         PointToPointPlanner &point_to_point_planner,
                         std::chrono::milliseconds time_budget) {

    auto si = point_to_point_planner.getPlanner()->getSpaceInformation();

    GoalApproachTable goal_samples = takeGoalSamples(si, goals, 10);

//    auto visitation_order = random_initial_solution(goal_samples);

    MultiGoalPlanResult result;

//    auto ss = si->getStateSpace()->as<DroneStateSpace>();
//
//    ompl::base::ScopedState start(si);
//
//    ompl::base::State *last_state = start.get();
//
//    for (const auto &visit: visitation_order) {
//        const auto &goal = goal_samples[visit.target_idx][visit.approach_idx];
//        auto path = point_to_point_planner.planToOmplState(MAX_TIME_PER_TARGET_SECONDS, last_state, goal->get());
//        if (path) {
//            auto traj = convertTrajectory(path.value(), robot);
//
//            result.segments.push_back(
//                    PointToPointPlanResult{
//                            traj.getLength(),
//                            traj,
//                            apples.apples[visit.target_idx].center
//                    }
//            );
//        }
//    }

    return result;
}



std::vector<GoalRegionPtr>
ApproachClustering::constructGoalRegions(const TreeScene &apples, const ompl::base::SpaceInformationPtr &si) {
    std::vector<GoalRegionPtr> goals;
    for (const Apple &apple: apples.apples) {
        goals.push_back(std::make_shared<DroneEndEffectorNearTarget>(si, GOAL_END_EFFECTOR_RADIUS, apple.center));
    }
    return goals;
}

std::string ApproachClustering::getName() {
    return "Approach Clustering";
}

ApproachClustering::ApproachClustering(size_t initialK) : initial_k(initialK) {}


