
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <boost/range/combine.hpp>
#include "../src/thread_pool.hpp"

#include "../utilities/experiment_utils.h"
#include "../DronePathLengthObjective.h"


double planFromStateToState(const PointToPointPair &pair, std::shared_ptr<ompl::geometric::PRMstar> &planner,
                            const ompl::base::OptimizationObjectivePtr &objective);

std::shared_ptr<ompl::geometric::PRMstar> allocPlanner(const moveit_msgs::PlanningScene &scene_msg,
                                                       const moveit::core::RobotModelPtr &drone,
                                                       const ompl::base::StateSamplerAllocator& ssa) {

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);
    state_space->setStateSamplerAllocator(ssa);
    state_space->setup();
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    return std::make_shared<ompl::geometric::PRMstar>(si);

}

int main(int argc, char **argv) {

    const double SOLVE_TIME = 20.0;

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage(appletree);

    auto alloc_objective = [](const auto &si) -> ompl::base::OptimizationObjectivePtr {
        return std::make_shared<DronePathLengthObjective>(si);
    };

//    const std::chrono::milliseconds planning_times[] = {
//            std::chrono::milliseconds(50),
//            std::chrono::milliseconds(100),
//            std::chrono::milliseconds(200),
//            std::chrono::milliseconds(500),
//            std::chrono::milliseconds(1000),
////            std::chrono::milliseconds(2000),
////            std::chrono::milliseconds(5000),
//    };

    auto planning_pairs = samplePlanningPairs(setupPlanningScene(scene_msg, drone), drone, apples, 100);

    Json::Value value;

    {
        auto planner = allocPlanner(scene_msg, drone,
                                    [](const ompl::base::StateSpace *state_space) -> ompl::base::StateSamplerPtr {
                                        return std::make_shared<DroneStateSampler>(state_space);
                                    });

        for (const auto &pair: planning_pairs) {
            value["keep_graph"].append(planFromStateToState(pair, planner,alloc_objective(planner->getSpaceInformation())));
        }
    }

    {
        auto planner = allocPlanner(scene_msg, drone,
                                    [](const ompl::base::StateSpace *state_space) -> ompl::base::StateSamplerPtr {
                                        return std::make_shared<DroneStateSampler>(state_space);
                                    });

        for (const auto &pair: planning_pairs) {
            value["keep_graph"].append(planFromStateToState(pair, planner,alloc_objective(planner->getSpaceInformation())));
        }
    }

    {
        for (const auto &pair: planning_pairs) {
            auto planner = allocPlanner(scene_msg, drone,
                                        [](const ompl::base::StateSpace *state_space) -> ompl::base::StateSamplerPtr {
                                            return std::make_shared<DroneStateSampler>(state_space);
                                        });
            value["clear_graph"].append(planFromStateToState(pair, planner, alloc_objective(planner->getSpaceInformation())));
        }
    }

    std::ofstream ofs;
    ofs.open("analysis/prm_clear_graph_comparison.json");
    ofs << value;
    ofs.close();

}

double planFromStateToState(const PointToPointPair &pair,
                            std::shared_ptr<ompl::geometric::PRMstar> &planner,
                            const ompl::base::OptimizationObjectivePtr &objective) {

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner->getSpaceInformation());
    pdef->setOptimizationObjective(objective);
    pdef->setStartAndGoalStates(pair.from_state.get(), pair.to_state.get());

    planner->setProblemDefinition(pdef);

    if (planner->solve(ompl::base::timedPlannerTerminationCondition(1.0)) ==
        ompl::base::PlannerStatus::EXACT_SOLUTION) {
        return pdef->getSolutionPath()->length();
    } else {
        return INFINITY;
    }
}
