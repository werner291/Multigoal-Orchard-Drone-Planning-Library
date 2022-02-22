
#include <execution>
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <boost/range/adaptor/transformed.hpp>
#include "../src/experiment_utils.h"

struct PointToPointPair {
    size_t from_target;
    std::shared_ptr<ompl::base::State> from_state;

    size_t to_target;
    std::shared_ptr<ompl::base::State> to_state;
};

std::vector<PointToPointPair>
samplePlanningPairs(const planning_scene::PlanningScenePtr &scene, const moveit::core::RobotModelPtr &drone,
                    const std::vector<Apple> &apples, const size_t num_samples) {

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);
    state_space->setup();

    std::default_random_engine rng;

    auto si = initSpaceInformation(scene, drone, state_space);
    auto goals = constructAppleGoals(si, apples);

    auto del_state = [state_space = state_space](ompl::base::State *st) { state_space->freeState(st); };

    return boost::copy_range<std::vector<PointToPointPair>>(
            boost::irange<size_t>(0, num_samples) |
            boost::adaptors::transformed(
                    [apples = apples, state_space = state_space, &rng, &del_state, &goals, &si](size_t i) {

                        auto[target_i, target_j] = generateIndexPairNoReplacement(rng, apples.size());

                        std::shared_ptr<ompl::base::State> from_state(state_space->allocState(), del_state);
                        goals[target_i]->sampleGoal(from_state.get());

                        std::shared_ptr<ompl::base::State> to_state(state_space->allocState(), del_state);
                        goals[target_j]->sampleGoal(to_state.get());

                        return PointToPointPair{target_i, from_state, target_j, to_state};

                    }));
}

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();
    auto scene = setupPlanningScene(scene_msg, drone);

    std::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &)> planner_allocators[] = {
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::RRTstar>(si); },
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::PRMstar>(si); },
            [](const auto &si) -> ompl::base::PlannerPtr { return std::make_shared<ompl::geometric::AITstar>(si); }
    };

    auto planning_pairs = samplePlanningPairs(scene, drone, apples, 10);

    std::vector<double> path_lengths(planning_pairs.size());

    std::transform(std::execution::par, planning_pairs.begin(), planning_pairs.end(), path_lengths.begin(),
                  [&](const PointToPointPair &pair) {
        for (const auto &planner_allocator: planner_allocators) {
            ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
            auto state_space = std::make_shared<DroneStateSpace>(spec);
            state_space->setup();
            auto si = initSpaceInformation(scene, drone, state_space);

            auto planner = planner_allocator(si);
            auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
            pdef->setStartAndGoalStates(pair.from_state.get(), pair.to_state.get());

            planner->setProblemDefinition(pdef);
            planner->solve(5.0);
        }
        return 0.0;
    });

    for (const auto &item : path_lengths) {
        std::cout << "Length " << item;
    }
    std::cout << std::endl;


}