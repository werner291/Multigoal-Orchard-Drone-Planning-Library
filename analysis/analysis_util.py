import pandas as pd
import json


def abbreviate(name):
    """
    The JSON file typically uses relatively long names for various dictionary keys and other strings for clarity,
    but these are fairly unreadable when displayed on a plot or a table. This function converts the long names to
    shorter abbreviations.

    :param name:    The long name to abbreviate.
    :return:        The abbreviated name, or the original name if no abbreviation is found.
    """

    return {
        'optimize_segments': 'sopt',
        'prm_build_time': 't_prm',
        'samples_per_goal': 'k',
        'timePerAppleSeconds': 'tpa',
        'apply_shellstate_optimization': 'shopt',
        'useImprovisedSampler': 'imp',
        'tryLuckyShots': 'l',
        'useCostConvergence': 'conv',
        'GreatCircle': 'GC'
    }.get(name, name)


run_variables = [
    'scene_name',
    'start_state'
]

stat_columns = {
    'final_path_length',
    'goals_visited',
    'length_per_goal',
    'napples',
    'run_time'
}


def load_run_results(filename):
    """
    Load a set of run results (produced by the run_planner_experiment method) from a JSON file.

    :param filename:    The name of the JSON file to load.
    :return:            A Pandas DataFrame containing the run results.
    """
    with open(filename) as f:
        paths_df = pd.json_normalize(json.load(f))

    paths_df['length_per_goal'] = paths_df['final_path_length'] / paths_df['goals_visited']

    other_columns = list(
        paths_df
        .columns
        .difference(stat_columns)
        .difference(run_variables)
    )

    paths_df.fillna('N/A', inplace=True)

    paths_df.set_index(run_variables + other_columns, inplace=True)

    return paths_df


def aggregate_run_results(results: pd.DataFrame):
    """
    Aggregate a dataframe of run results into per-run average and standard deviation statistics.

    :param results:     The dataframe of run results (see load_run_results).
    :return:            A dataframe of aggregated run results.
    """

    keys = results.index.names.difference(run_variables)

    return results.groupby(keys).agg(['mean', 'sem'])


def load_aggregated(filename):
    """Load and aggregate results."""
    results = load_run_results(filename)
    return aggregate_run_results(results)
