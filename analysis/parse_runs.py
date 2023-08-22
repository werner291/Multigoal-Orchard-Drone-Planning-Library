import json
import pandas as pd
import matplotlib.pyplot as plt
import itertools


def flatten_run_dict(run_dict):
    """
    Flatten the given run dictionary into a flat dictionary that excludes the "segments".

    Args:
        run_dict (dict): A dictionary representing the run object.

    Returns:
        dict: A flat dictionary containing the information from the run object.
    """
    flat_dict = {}
    parameters_dict = run_dict["parameters"]
    problem_dict = parameters_dict["problem"]
    result_dict = run_dict["result"]
    initial_knowledge_dict = result_dict["initial_knowledge"]

    flat_dict.update(
        {
            "planner": parameters_dict["planner"],
            "tree_model": problem_dict["tree_model"],
            "n_discoverable": problem_dict["n_discoverable"],
            "n_false": problem_dict["n_false"],
            "n_given": problem_dict["n_given"],
            "n_total": problem_dict["n_total"],
            "visibility_model": problem_dict["visibility_model"],
            "discoverable": initial_knowledge_dict["discoverable"],
            "false_positives": initial_knowledge_dict["false_positives"],
            "known_unvisited": initial_knowledge_dict["known_unvisited"],
            "total": initial_knowledge_dict["total"],
            "visited": initial_knowledge_dict["visited"],
            "n_visited": result_dict["n_visited"],
            "time": result_dict["time"],
            "timeout": result_dict["timeout"],
            "total_experiment_runtime": result_dict["total_experiment_runtime"],
            "total_path_length": result_dict["total_path_length"],
        }
    )

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
            - end_event (str): The type of event that ended the segment.
    """

    if "discovery_type" in segment_dict["end_event"]:
        if segment_dict["end_event"]["discovery_type"] == "FOUND_FAKE_GOAL":
            end_event = "FOUND_FAKE_GOAL"

            # Check the ID of the fake goal against the original planned goal.
            if segment_dict["end_event"]["goal_id"] == segment_dict["planned_goal"]:
                end_event = "FOUND_FAKE_GOAL (planned)"

        elif segment_dict["end_event"]["discovery_type"] == "FOUND_NEW_GOAL":
            end_event = "FOUND_NEW_GOAL"
        else:
            raise ValueError("Unknown discovery type: {}".format(segment_dict["end_event"]["discovery_type"]))
    elif segment_dict["end_event"]["type"] == "PathEnd":
        end_event = "SUCCESSFUL_VISIT"
    else:
        raise ValueError("Unknown end event type: {}".format(segment_dict["end_event"]["type"]))

    return {
        "path_length": segment_dict["path_length"],
        "time": segment_dict["time"],
        "end_event": end_event,
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
        if run_dict is None:
            continue

        run_data = flatten_run_dict(run_dict)
        runs_data.append(run_data)

        if "solution_segments" in run_dict["result"]:
            segments = []

            for segment_dict in run_dict["result"]["solution_segments"]:
                segment_data = flatten_segment_dict(segment_dict)
                segments.append(segment_data)

            segments_data.append(segments)

    runs_df = pd.DataFrame(runs_data)
    runs_df["length_per_visited"] = runs_df["total_path_length"] / runs_df["n_visited"]
    runs_df["time_per_visited"] = runs_df["time"] / runs_df["n_visited"]
    segments_df_list = [pd.DataFrame(run_segments) for run_segments in segments_data]

    if segments_df_list:
        return runs_df, segments_df_list
    else:
        return runs_df


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
            - end_event (str): the type of event that ended the segment

    Raises:
    - ValueError: if the file at the given filepath is not a valid JSON array
    """
    data = load_json(filename)
    return json_to_dataframe(data)


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

    scenario_params = ["n_total", "n_discoverable", "visibility_model"]

    if metrics is None:
        metrics = ["length_per_visited", "n_visited"]

    n_scenarios = len(runs.groupby(scenario_params))

    # Create a grid of subplots with the dimensions based on the number of metrics and scenario_params
    fig, axes = plt.subplots(
        n_scenarios, len(metrics), figsize=(5 * len(scenario_params), 8 * n_scenarios)
    )

    # Loop through runs grouped by scenario_params, and plot each metric in a separate row
    for ((n, nd, v), df), ax_row in zip(runs.groupby(scenario_params), axes):
        # Loop through the metrics and plot boxplots for each metric and group
        for column, ax in zip(metrics, ax_row):
            # Create a boxplot for the current metric and group
            df.boxplot(column, rot=90, by="planner", ax=ax)

            # Set the title, x-axis label, and y-axis label for the current subplot
            ax.set_title("{}, n={}, d={}, v={}".format(column, n, nd, v))
            ax.set_xlabel("Planner")
            ax.set_ylabel(column)

            # Set the y-axis lower limit to 0
            ax.set_ylim(bottom=0)

    # Adjust the layout of the subplots for better spacing
    plt.tight_layout()

    # Display the generated boxplots
    plt.show()


def df_zipgroupby(dfs, key):
    """
    Group rows from multiple dataframes by the given key, yielding pairs pf the form (value, groups), where value is
    the value of the key column for that group, and groups is a list of dataframes, one for each input dataframe,

    :param dfs:     the dataframes to be grouped
    :param key:     the name of the column to be used as the key
    :return:        an iterator over (value, groups) pairs
    """

    for groups in itertools.zip_longest(*[df.groupby(key) for df in dfs]):
        group_key = groups[0][0]
        group_dfs = [group[1] for group in groups]
        yield group_key, group_dfs


def subplots_from_groupings(df, rows_variable, columns_variable, figsize=(12, 8)):
    """
    Generate a grid of subplots based on a pair of column values in the dataframe(s).

    The names must be available as named columns in the dataframe(s).

    The unique values of the column designated by the columns_variable will be used to generate the columns of the grid.
    Similarly, the unique values of the column designated by the rows_variable will be used to generate the rows of the grid.

    One or more dataframes can be passed to this function. If multiple dataframes are passed, each must have the given
    columns_variable and rows_variable available as named columns.

    :param df: one or more dataframes to be plotted
    :param columns_variable: the name of the column to be used for the columns of the grid
    :param rows_variable: the name of the column to be used for the rows of the grid
    :param figsize: the size of the figure to be generated
    """

    ## Depending on whether there is one or more dataframes, we need to handle the subplots differently
    if type(df) is pd.DataFrame:
        n_rows = df.groupby(columns_variable)[rows_variable].nunique().max()
        n_cols = df.groupby(rows_variable)[columns_variable].nunique().max()

        fig, axs = plt.subplots(
            n_rows,
            n_cols,
            figsize=figsize,
        )

        for row_i, (row_name, group) in enumerate(df.groupby(rows_variable)):
            for col_i, (col_name, group_df) in enumerate(
                    group.groupby(columns_variable)
            ):
                yield (row_i, col_i), (row_name, col_name), group_df, axs[row_i, col_i]

    else:
        n_rows = max([df.groupby(columns_variable)[rows_variable].nunique().max() for df in df])
        n_cols = max([df.groupby(rows_variable)[columns_variable].nunique().max() for df in df])

        fig, axs = plt.subplots(
            n_rows,
            n_cols,
            figsize=figsize,
        )

        for row_i, (row_name, row_dfs) in enumerate(df_zipgroupby(df, rows_variable)):
            for col_i, (col_name, col_dfs) in enumerate(df_zipgroupby(row_dfs, columns_variable)):
                yield (row_i, col_i), (row_name, col_name), col_dfs, axs[row_i][col_i]


def merge_segments(segments):
    '''
    Merge a list of segments into a single dataframe, adding a column for the segment index and run index.

    :param segments:    a list of dataframes, each representing a segment
    :return:            a single dataframe containing all segments
    '''
    segs = []
    for i,seg in enumerate(segments):
        seg = seg.reset_index().rename(columns={'index': 'segment_index'})
        seg['run_index'] = i
        segs.append(seg)
    return pd.concat(segs)