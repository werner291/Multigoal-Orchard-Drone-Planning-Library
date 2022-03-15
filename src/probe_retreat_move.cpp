
#include <boost/range/irange.hpp>
#include <utility>
#include <ompl/geometric/PathSimplifier.h>
#include "probe_retreat_move.h"



ompl::geometric::PathGeometric
plan_probe_retreat_slide(const std::vector<Apple> &apples_in_order, const ompl::base::State *initial_state,
                         const ompl::base::SpaceInformationPtr &si,
                         const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State *,
                                                                                           ompl::base::State *)> &plan_state_to_state,
                         const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State *,
                                                                                           const Apple &apple)> &plan_state_to_apple,
                         const Highway<Apple>& highway,
                         bool simplify) {


    struct AppleApproach {
        Apple apple;
        ompl::geometric::PathGeometric approach;
    };

    std::vector<AppleApproach> approaches;
    for (const Apple& apple : apples_in_order) {

        ompl::base::ScopedState state_outside_tree_for_apple(si);
        highway.on_ramp(apple, state_outside_tree_for_apple.get());

        if (auto approach = plan_state_to_apple(state_outside_tree_for_apple.get(), apple)) {
            approaches.push_back({apple, *approach});
        }
    }

    ompl::geometric::PathGeometric full_path(si, initial_state);

    for (size_t approach_idx : boost::irange<size_t>(1,approaches.size())) {
        ompl::geometric::PathGeometric apple_to_apple(approaches[approach_idx-1]);
        apple_to_apple.reverse();
        apple_to_apple.append(*highway.plan_state_to_state(approaches[approach_idx-1].getState(0),approaches[approach_idx].getState(0)));
        apple_to_apple.append(approaches[approach_idx]);

        if (simplify) {
            ompl::geometric::PathSimplifier ps(si);
            ps.simplifyMax(apple_to_apple);
            ps.shortcutPath(apple_to_apple, 100, 100);
        }

        full_path.append(apple_to_apple);
    }

    return full_path;

}

