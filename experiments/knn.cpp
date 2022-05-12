
#include "../src/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../src/greatcircle.h"
#include "../src/probe_retreat_move.h"
#include <range/v3/all.hpp>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <fstream>

using namespace ranges;
using namespace std;
namespace og = ompl::geometric;

class NewMultiGoalPlanner {

public:
    struct PathSegment {
        size_t to_goal_id_;
        ompl::geometric::PathGeometric path_;
    };

    struct PlanResult {
        std::vector<PathSegment> segments_;
    };

    typedef const std::function<std::optional<og::PathGeometric>(const ompl::base::State *, const ompl::base::Goal *)> StateToGoalFn;
    typedef const std::function<std::optional<og::PathGeometric>(const ompl::base::State *, const ompl::base::State *)> StateToStateFn;

    virtual PlanResult plan(const ompl::base::SpaceInformationPtr& si,
                            const ompl::base::State* start,
                            const std::vector<ompl::base::Goal*> &goals,
                            StateToGoalFn plan_state_to_goal,
                            StateToStateFn plan_state_to_state) = 0;
};

class NewKnnPlanner : public NewMultiGoalPlanner {

    const size_t START_STATE_REF = SIZE_MAX;

    std::function<double(const ompl::base::State* , const ompl::base::Goal* )> state_to_goal_heuristic_distance_;
public:
    NewKnnPlanner(
            const function<double(const ompl::base::State *, const ompl::base::Goal *)> &stateToGoalHeuristicDistance,
            const function<double(const ompl::base::Goal *, const ompl::base::Goal *)> &goalToGoalHeuristicDistance,
            size_t k) : state_to_goal_heuristic_distance_(stateToGoalHeuristicDistance),
                        goal_to_goal_heuristic_distance_(goalToGoalHeuristicDistance), k(k) {}

private:
    std::function<double(const ompl::base::Goal* , const ompl::base::Goal* )> goal_to_goal_heuristic_distance_;
    size_t k;

    typedef std::pair<size_t,const ompl::base::Goal *> GoalIdAndGoal;

    typedef std::variant<
            const ompl::base::State *,
            GoalIdAndGoal
    > StateOrGoal;


public:
    ompl::NearestNeighborsGNAT<StateOrGoal>
    buildGNAT(const ompl::base::State *start, const vector<ompl::base::Goal *> &goals) const {

        ompl::NearestNeighborsGNAT<StateOrGoal> gnat;

        gnat.setDistanceFunction([&](const StateOrGoal& a, const StateOrGoal& b) -> double {
            if (a.index() == 0) {
                assert(b.index() == 1);
                return state_to_goal_heuristic_distance_(get<const ompl::base::State *>(a), get<GoalIdAndGoal>(b).second);
            }

            if (b.index() == 0) {
                assert(a.index() == 1);
                return state_to_goal_heuristic_distance_(get<const ompl::base::State *>(b), get<GoalIdAndGoal>(a).second);
            }

            return goal_to_goal_heuristic_distance_(get<GoalIdAndGoal>(a).second, get<GoalIdAndGoal>(b).second);
        });

        gnat.add(goals | views::enumerate | views::transform([&](auto&& pair) -> StateOrGoal {
            return {std::make_pair(pair.first, pair.second)};
        }) | to_vector);

        return gnat;
    }

    PlanResult plan(const ompl::base::SpaceInformationPtr& si,
                    const ompl::base::State* start,
                    const std::vector<ompl::base::Goal*> &goals,
                    StateToGoalFn plan_state_to_goal,
                    StateToStateFn plan_state_to_state) override {

        auto gnat = buildGNAT(start, goals);

        std::vector<PathSegment> segments;

        size_t goals_visited = 0;

        ompl::base::State* last_state;

        while (gnat.size() > 0) {

            std::vector<StateOrGoal> nearest_k;

            // Find the k nearest apples.
            gnat.nearestK({last_state}, k, nearest_k);

            std::vector<pair<StateOrGoal ,og::PathGeometric>> paths;

            // For all, plan a path
            for (auto &apple: nearest_k) {

                if (auto result = plan_state_to_goal(last_state, get<GoalIdAndGoal>(apple).second)) {
                    paths.emplace_back(apple,result.value());
                }
            }

            if (paths.empty()) {
                gnat.remove(nearest_k[0]);
            } else {
                // Pick the shortest path
                auto shortest = paths[0];
                for (auto &path: paths) {
                    if (path.second.length() < shortest.second.length()) {
                        shortest = path;
                    }
                }
                gnat.remove(shortest.first);
                segments.push_back({get<GoalIdAndGoal>(shortest.first).first, shortest.second});
                last_state = shortest.second.getState(shortest.second.getStateCount() - 1);

                goals_visited += 1;
            }
        }

        return PlanResult { segments };
    }
};

int main(int argc, char **argv) {

    // Load the drone model
    auto drone = loadRobotModel();

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    const size_t NUM_RUNS = 1;

    thread_pool pool(8);

    const auto run_indices = ranges::views::iota(0, (int) NUM_RUNS);

    const auto ks = ranges::views::iota(1,3);

    const double PLAN_TIME_PER_APPLE_SECONDS = 5.0;

    GreatCircleMetric gc_metric(scene_info.sphere_center);
    std::function<double(Eigen::Vector3d ,Eigen::Vector3d )> distance_fns[2] = {
        [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return gc_metric.measure(a, b); },
        [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return (a - b).norm(); },
    };

    // Mutex
    mutex result_mutex;
    Json::Value statistics;

    auto cartesian = ranges::views::cartesian_product(run_indices, ks, distance_fns) | to_vector;

    for (auto [run_i,k,distance_fn]: cartesian) {

        pool.push_task([&,run_i = run_i,k=k,distance_fn=distance_fn]() {

            // Send all the parameters to cout
            cout << "run_i: " << run_i << " k: " << k << endl;

            // initialize the state space and such
            const auto& [state_space, si, objective] = loadContext(drone, scene_info.scene_msg);

            auto start_state_moveit = stateOutsideTree(drone);

            ompl::base::ScopedState<> start_state(si);

            state_space->copyToOMPLState(start_state.get(), start_state_moveit);

            NewKnnPlanner planner



            RobotPath full_path_moveit = omplPathToRobotPath(full_path);

            {
                scoped_lock lock(result_mutex);


                Json::Value run_stats;

                run_stats["run_i"] = run_i;
                run_stats["final_path_length"] = full_path_moveit.length();
                run_stats["goals_visited"] = (int) goals_visited;
                statistics["path_lengths"].append(run_stats);

                cout << "Done " << statistics["path_lengths"].size() << " out of " << cartesian.size() << endl;
            }
        });
    }

    pool.wait_for_tasks();

    std::ofstream ofs;
    ofs.open("analysis/knn_results.json");
    ofs << statistics;
    ofs.close();

    return 0;

}