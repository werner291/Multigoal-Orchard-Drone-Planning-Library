use std::iter::once;
use itertools::Itertools;



trait Goal {

}

trait Path : Clone {
    fn reverse(self) -> Self;

    fn concatenate(self, other: Self) -> Self;

    fn singleton(state: impl State) -> Self;
}

trait ShellPoint : Copy {

}

#[derive(Clone, Debug)]
struct ApproachPath<P: Path, SP: ShellPoint> {
    path: P,
    shell_point: SP,
}

enum PlanPathStaticError {
    InitialApproachFailed
}

fn reorder_vec<T>(vec: Vec<T>, indices: Vec<usize>) -> Vec<T> {

    let mut optionalized = vec.into_iter().map(Option::Some).collect::<Vec<_>>();

    return indices.iter().map(|index| {
        optionalized[*index].take().expect("Index was not unique.")
    }).collect();

}

trait TSPOrderStrategy<T> {

    fn order(&self, initial: T, goal: &Vec<T>) -> Vec<usize>;

}

fn plan_path_static_goalset<
    St: State,
    G: Goal,
    P : Path,
    PathOptimizer: Fn(&P) -> P,
    SP: ShellPoint,
    PredictShellPathLength: Fn(&SP, &SP) -> f64,
    PlanShellPath: Fn(SP, SP) -> P,
    PlanApproachToGoalFn : Fn(&G) -> Option<ApproachPath<P, SP>>,
    PlanApproachToStateFn : Fn(&St) -> Option<ApproachPath<P, SP>>,
    TSPOrder: TSPOrderStrategy<SP>,
>(
    initial_state: &St,
    goal_set: &Vec<G>,
    plan_approach_to_goal: PlanApproachToGoalFn,
    plan_approach_to_state: PlanApproachToStateFn,
    plan_shell_path: PlanShellPath,
    tsp_planner: TSPOrder,
    path_optimizer: PathOptimizer,
) -> Result<P, PlanPathStaticError> {

    let approach_paths = goal_set
        .iter()
        .filter_map(|goal| plan_approach_to_goal(goal))
        .collect::<Vec<ApproachPath<P, SP>>>();

    let initial_approach = plan_approach_to_state(initial_state).ok_or(PlanPathStaticError::InitialApproachFailed)?;

    let order = tsp_planner.order(
        initial_approach.shell_point,
                            &approach_paths
                                .iter()
                                .map(|approach_path| approach_path.shell_point)
                                .collect());

    let approach_paths_in_order = reorder_vec(approach_paths, order);

    let goal_to_goal_paths =
        once(initial_approach)
            .chain(approach_paths_in_order)
            .tuple_windows()
            .map(|(approach_path_1, approach_path_2)| {

                let shell_path = plan_shell_path(approach_path_1.shell_point, approach_path_2.shell_point);

                let assembly =
                    approach_path_1.path.clone().reverse()
                        .concatenate(shell_path)
                        .concatenate(approach_path_2.path.clone());

                path_optimizer(&assembly)
            });

    let assembled = goal_to_goal_paths
            .fold(Path::singleton(initial_state.clone()), |path:P, goal_to_goal_path:P| path.concatenate(goal_to_goal_path));

    Ok(assembled)
}