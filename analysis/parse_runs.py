import json
import pandas as pd
import matplotlib.pyplot as plt


def flatten_run_dict(run_dict):
    """
    Flatten the given run dictionary into a flat dictionary that excludes the "segments".

    Args:
        run_dict (dict): A dictionary representing the run object.

    Returns:
        dict: A flat dictionary containing the information from the run object.
    """
    flat_dict = {}
    parameters_dict = run_dict['parameters']
    problem_dict = parameters_dict['problem']
    result_dict = run_dict['result']
    initial_knowledge_dict = result_dict['initial_knowledge']

    flat_dict.update({
        'planner': parameters_dict['planner'],
        'n_discoverable': problem_dict['n_discoverable'],
        'n_false': problem_dict['n_false'],
        'n_given': problem_dict['n_given'],
        'n_total': problem_dict['n_total'],
        'visibility_model': problem_dict['visibility_model'],
        'discoverable': initial_knowledge_dict['discoverable'],
        'false_positives': initial_knowledge_dict['false_positives'],
        'known_unvisited': initial_knowledge_dict['known_unvisited'],
        'total': initial_knowledge_dict['total'],
        'visited': initial_knowledge_dict['visited'],
        'n_visited': result_dict['n_visited'],
        'time': result_dict['time'],
        'total_experiment_runtime': result_dict['total_experiment_runtime'],
        'total_path_length': result_dict['total_path_length'],
    })

    return flat_dict


def flatten_segment_dict(segment_dict):
    """
    Takes a dictionary representing a segment in a planner run and returns a flattened version.

    Parameters:
        - segment_dict (dict): A dictionary representing a segment of a planner run.

    Returns:
        - A dictionary containing the information in a flat format.
          The dictionary will have the following keys:
            - path_length (float): The length of the path taken in the segment.
            - time (int): The time taken in the segment.
            - goals_discovered (int): The number of new goals discovered in the segment.
            - goals_visited (int): The number of goals visited in the segment.
    """
    goals_discovered = 0
    if 'discovery_type' in segment_dict['end_event']:
        goals_discovered = 1

    goals_visited = 0
    if 'goals_visited' in segment_dict['end_event']:
        goals_visited = len(segment_dict['end_event']['goals_visited'])

    return {
        'path_length': segment_dict['path_length'],
        'time': segment_dict['time'],
        'goals_discovered': goals_discovered,
        'goals_visited': goals_visited
    }


def json_to_dataframe(json_data):
    """
    Converts a JSON object to a pair of pandas DataFrames: one for the runs (excluding segments), and one for the segments
    (with a foreign key to the first DataFrame).

    Args:
        json_data (str or dict): JSON data to convert to a DataFrame.

    Returns:
        tuple of pandas.DataFrame: A tuple containing two DataFrames: the first one has a row for each run (excluding
        segments), and the second one has a row for each segment (with a foreign key to the first DataFrame).
    """
    runs_data = []
    segments_data = []

    for i, run_dict in enumerate(json_data):
        run_data = flatten_run_dict(run_dict)
        runs_data.append(run_data)

        segments = []

        for segment_dict in run_dict['result']['solution_segments']:
            segment_data = flatten_segment_dict(segment_dict)
            segments.append(segment_data)

        segments_data.append(segments)

    runs_df = pd.DataFrame(runs_data)
    runs_df['length_per_visited'] = runs_df['total_path_length'] / runs_df['n_visited']
    runs_df['time_per_visited'] = runs_df['time'] / runs_df['n_visited']
    segments_df_list = [pd.DataFrame(run_segments) for run_segments in segments_data]

    return runs_df, segments_df_list


def load_json(filename):
    with open(filename) as f:
        return json.load(f)


def load_and_process_json(filename):
    """
    Loads a JSON file from the given filepath and converts its contents into two pandas DataFrames:
    one for the main run data (excluding the segments), and one for the segments themselves.

    Args:
    - filepath (str): the path to the JSON file to be loaded

    Returns:
    - A tuple of two pandas DataFrames:
        - The first DataFrame has one row per run and includes columns for:
            - planner (str): the planner used for the run
            - n_discoverable (int): the number of discoverable goals for the problem in the run
            - n_false (int): the number of false goals for the problem in the run
            - n_given (int): the number of given goals for the problem in the run
            - n_total (int): the total number of goals for the problem in the run
            - visibility_model (str): the visibility model used for the run
            - discoverable (int): the number of goals that were discovered during the run
            - visited (int): the number of goals that were visited during the run
            - initial_known_unvisited (int): the number of goals that were known but not visited at the start of the run
            - initial_false_positives (int): the number of false positives that were initially present
            - initial_total (int): the total number of goals that were present at the start of the run
            - time (int): the total runtime of the run in milliseconds
            - total_experiment_runtime (int): the total runtime of the experiment in milliseconds
            - total_path_length (float): the total length of the path taken during the run
        - The second DataFrame has one row per segment and includes columns for:
            - run_id (int): a foreign key linking to the corresponding row in the first DataFrame
            - path_length (float): the length of the path taken during the segment
            - time (int): the runtime of the segment in milliseconds
            - goals_discovered (int): the number of new goals discovered during the segment
            - goals_visited (int): the number of goals visited during the segment

    Raises:
    - ValueError: if the file at the given filepath is not a valid JSON array
    """
    data = load_json(filename)
    runs_df, segments_df = json_to_dataframe(data)
    return runs_df, segments_df


def make_boxplots(runs, metrics=None):
    """
    Generates boxplots for given metrics and scenario parameters on a set of runs.

    This function generates a 2D grid of boxplots, with rows representing different
    metrics and columns representing different combinations of scenario parameters.
    Each boxplot shows the distribution of a given metric across different planner
    types.

    Parameters:
    ----------
    runs: pandas.DataFrame
        A DataFrame containing the run data, with columns for metrics, planner, and
        scenario parameters.
    metrics: list of str, optional (default=['length_per_visited', 'n_visited'])
        A list of metric column names to plot.

    Returns:
    -------
    None
    """

    scenario_params = ['n_total', 'n_discoverable', 'visibility_model']

    if metrics is None:
        metrics = ['length_per_visited', 'n_visited']

    n_scenarios = len(runs.groupby(scenario_params))

    # Create a grid of subplots with the dimensions based on the number of metrics and scenario_params
    fig, axes = plt.subplots(n_scenarios, len(metrics), figsize=(5 * len(scenario_params), 8 * n_scenarios))

    # Loop through runs grouped by scenario_params, and plot each metric in a separate row
    for (((n, nd, v), df), ax_row) in zip(runs.groupby(scenario_params), axes):
        # Loop through the metrics and plot boxplots for each metric and group
        for (column, ax) in zip(metrics, ax_row):
            # Create a boxplot for the current metric and group
            df.boxplot(column, rot=90, by='planner', ax=ax)

            # Set the title, x-axis label, and y-axis label for the current subplot
            ax.set_title('{}, n={}, d={}, v={}'.format(column, n, nd, v))
            ax.set_xlabel('Planner')
            ax.set_ylabel(column)

            # Set the y-axis lower limit to 0
            ax.set_ylim(bottom=0)

    # Adjust the layout of the subplots for better spacing
    plt.tight_layout()

    # Display the generated boxplots
    plt.show()
