#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.

import glob
import json
import os


def load_benchmark_results(benchmark_name):
    """
    Finds the latest JSON file with the given benchmark name prefix in the current working directory,
    prints metadata about the file, and returns the 'results' value from the JSON data.

    Parameters:
    benchmark_name (str): The prefix of the benchmark files to search for.

    Returns:
    dict: The 'results' value from the latest benchmark JSON file.
    """
    list_of_files = glob.glob(f'./{benchmark_name}*.json')
    if not list_of_files:
        raise FileNotFoundError(f"No {benchmark_name} files found in the current working directory.")
    latest_file = max(list_of_files, key=os.path.getctime)

    with open(latest_file, 'r') as file:
        data = json.load(file)

    print(f"Using file: {latest_file}")
    print(f"Metadata: commit={data.get('commit')}, duration_ms={data.get('duration_ms')}, finished_at={data.get('finished_at')}")

    return data['results']
