
#ifndef NEW_PLANNERS_SHELLPATHPLANNER_H
#define NEW_PLANNERS_SHELLPATHPLANNER_H

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>
#include "MultiGoalPlanner.h"
#include "../SphereShell.h"
#include "../DistanceHeuristics.h"
#include "../planning_scene_diff_message.h"
#include "../DronePathLengthObjective.h"
#include "../general_utilities.h"
#include "../traveling_salesman.h"
#include "../probe_retreat_move.h"
#include "../ExperimentVisualTools.h"

template<typename ShellPoint>
class ShellPathPlanner : public MultiGoalPlanner {

public:
	class ShellBuilder {

	public:
		virtual std::shared_ptr<OMPLSphereShellWrapper<ShellPoint>> buildShell(
				const AppleTreePlanningScene &scene_info,
				const ompl::base::SpaceInformationPtr &si)const  = 0;

		virtual Json::Value parameters() const = 0;

	};

private:
	std::shared_ptr<const ShellBuilder> shell_builder;
    std::shared_ptr<SingleGoalPlannerMethods> methods;

    bool apply_shellstate_optimization;

public:

	ShellPathPlanner(bool applyShellstateOptimization,
									   std::shared_ptr<SingleGoalPlannerMethods> methods,
									   std::shared_ptr<const ShellBuilder>  shellBuilder) :
			apply_shellstate_optimization(applyShellstateOptimization),
			methods(std::move(methods)), shell_builder(std::move(shellBuilder)) {}

	MultiGoalPlanner::PlanResult plan(
			const ompl::base::SpaceInformationPtr &si,
			const ompl::base::State *start,
			const std::vector<ompl::base::GoalPtr> &goals,
			const AppleTreePlanningScene &planning_scene,
			ompl::base::PlannerTerminationCondition& ptc) {

		auto shell = shell_builder->buildShell(planning_scene, si);

		rclcpp::init(0, nullptr);
		auto evt = std::make_shared<ExperimentVisualTools>();
		evt->publishPlanningScene(planning_scene.scene_msg);

		std::vector<ompl::base::ScopedState<>> shell_states;

		for (auto &goal : goals) {
			shell_states.emplace_back(si);
			shell->state_on_shell(shell->project(goal.get()), shell_states.back().get());
		}

		evt->pincushion(
				ranges::views::zip(goals, shell_states)
					| ranges::views::transform([&](const auto pair) {
						return std::make_pair(pair.first.get(), pair.second.get());
					})
					| ranges::to_vector,
				si->getStateSpace().get(),
				"projections");

		auto approaches = planApproaches(si, goals, *shell, ptc);

		PlanResult result {{}};

		if (approaches.empty()) {
			return result;
		}

		auto ordering = computeApproachOrdering(start, goals, approaches, *shell);

		auto first_approach = planFirstApproach(start, approaches[ordering[0]].second);

		if (!first_approach) {
			return result;
		}

		PlanResult fullPath = assembleFullPath(si, goals, *shell, approaches, ordering, result, *first_approach, false);

		evt->publishPath(si, "result_path", fullPath.combined());
		
		rclcpp::spin(evt);

		return fullPath;
	}

	MultiGoalPlanner::PlanResult assembleFullPath(const ompl::base::SpaceInformationPtr &si,
												  const std::vector<ompl::base::GoalPtr> &goals,
												  OMPLSphereShellWrapper<ShellPoint> &ompl_shell,
												  const std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> &approaches,
												  const std::vector<size_t> &ordering,
												  MultiGoalPlanner::PlanResult &result,
												  ompl::geometric::PathGeometric &initial_approach,
												  bool optimize_segments) const {

		result.segments.push_back({approaches[ordering[0]].first, initial_approach});

		for (size_t i = 1; i < ordering.size(); ++i) {

			ompl::geometric::PathGeometric goal_to_goal(si);

			auto segment_path = retreat_move_probe(
					goals,
					ompl_shell,
					result,
					goal_to_goal,
					approaches[ordering[i - 1]],
					approaches[ordering[i]]
			);

			if (optimize_segments) {
				segment_path = optimize(segment_path, std::make_shared<DronePathLengthObjective>(si), si);
			}

			result.segments.push_back({
											  approaches[ordering[0]].first,
											  segment_path
									  });

		}

		return result;
	}

	ompl::geometric::PathGeometric
	retreat_move_probe(const std::vector<ompl::base::GoalPtr> &goals, OMPLSphereShellWrapper<ShellPoint> &ompl_shell,
										 MultiGoalPlanner::PlanResult &result,
										 ompl::geometric::PathGeometric &goal_to_goal,
										 const std::pair<size_t, ompl::geometric::PathGeometric> &approach_a,
										 const std::pair<size_t, ompl::geometric::PathGeometric> &approach_b) const {

		goal_to_goal.append(approach_a.second);
		goal_to_goal.reverse();

		auto a = goals[approach_a.first].get();
		auto b = goals[approach_b.first].get();

		goal_to_goal.append(ompl_shell.path_on_shell(a,b));

		goal_to_goal.append(approach_b.second);

		return goal_to_goal;
	}

	std::optional<ompl::geometric::PathGeometric>
	planFirstApproach(const ompl::base::State *start,
										ompl::geometric::PathGeometric &approach_path) {

		auto start_to_shell = methods->state_to_state(
				start,
				approach_path.getState(0)
		);

		if (start_to_shell) {
			start_to_shell->append(approach_path);
		}

		return start_to_shell;
	}

	std::vector<size_t> computeApproachOrdering(
			const ompl::base::State *start,
			const std::vector<ompl::base::GoalPtr> &goals,
			const std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> &approaches,
			const OMPLSphereShellWrapper<ShellPoint>& shell) const {

		return tsp_open_end(
				[&](auto i) {
					return shell.predict_path_length(start, goals[approaches[i].first].get());
				},
				[&](auto i, auto j) {
					return shell.predict_path_length(
							goals[approaches[i].first].get(),
							goals[approaches[j].first].get()
					);
				},
				approaches.size()
		);
	}

	std::vector<std::pair<size_t, ompl::geometric::PathGeometric>>
	planApproaches(const ompl::base::SpaceInformationPtr &si,
									 const std::vector<ompl::base::GoalPtr> &goals,
									 const OMPLSphereShellWrapper<ShellPoint> &ompl_shell,
									 ompl::base::PlannerTerminationCondition &ptc) const {

		std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> approaches;

		for (const auto& [goal_i, goal] : goals | ranges::views::enumerate) {
			if (auto approach = planApproachForGoal(si, ompl_shell, goal)) {
				assert(approach->getStateCount() > 0);
				approaches.emplace_back(
						goal_i,
						*approach
				);
			}

			checkPtc(ptc);

		}

		return approaches;
	}

	std::optional<ompl::geometric::PathGeometric> planApproachForGoal(
			const ompl::base::SpaceInformationPtr &si,
			const OMPLSphereShellWrapper<ShellPoint> &ompl_shell,
			const ompl::base::GoalPtr &goal) const {

		ompl::base::ScopedState shell_state(si);
		ompl_shell.state_on_shell(goal.get(), shell_state.get());

		auto approach_path = methods->state_to_goal(shell_state.get(), goal);

		if (apply_shellstate_optimization && approach_path) {
			*approach_path = optimizeExit(
					goal.get(),
					*approach_path,
					std::make_shared<DronePathLengthObjective>(si),
					ompl_shell,
					si
			);
		}

		return approach_path;
	}

	Json::Value parameters() const override {
		Json::Value result;

		result["shell_builder_params"] = shell_builder->parameters();
		result["apply_shellstate_optimization"] = apply_shellstate_optimization;
		result["ptp"] = methods->parameters();

		return result;
	}

	std::string name() const {
		return "ShellPathPlanner";
	}

};

class PaddedSphereShellAroundLeavesBuilder : public ShellPathPlanner<Eigen::Vector3d>::ShellBuilder {

	double padding;
public:
	PaddedSphereShellAroundLeavesBuilder(double padding = 0.1);

public:
	std::shared_ptr<OMPLSphereShellWrapper<Eigen::Vector3d>>
	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) const override;

	[[nodiscard]] Json::Value parameters() const override;

};

#endif //NEW_PLANNERS_SHELLPATHPLANNER_H
