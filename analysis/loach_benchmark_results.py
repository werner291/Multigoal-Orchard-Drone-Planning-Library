#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.

import glob
import json
import os
import argparse


def load_benchmark_results_file(file_path):
    """
    Load benchmark results from a JSON file.

    Args:
        file_path (str): The path to the JSON file containing benchmark results.

    Returns:
        dict: The results contained in the JSON file.
    """
    with open(file_path, 'r') as file:
        data = json.load(file)

    print(f"Using file: {file_path}")
    print(f"Metadata: commit={data.get('commit')}, "
          f"duration_ms={data.get('duration_ms')}, "
          f"finished_at={data.get('finished_at')}")

    return data['results']


def load_benchmark_results(benchmark_name, description):
    """
    Load benchmark results, either from a specified file or the latest benchmark file.

    Args:
        benchmark_name (str): The name of the benchmark.
        description (str): The description for the argument parser.

    Returns:
        dict: The benchmark results.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('file_path', nargs='?', type=str, help='Path to the JSON file containing benchmark results.')
    args = parser.parse_args()

    if args.file_path:
        return load_benchmark_results_file(args.file_path)
    else:
        return load_benchmark_results_file(lookup_latest_benchmark_file(benchmark_name))


def lookup_latest_benchmark_file(benchmark_name):
    """
    Find the latest benchmark file by modification time.

    Args:
        benchmark_name (str): The name of the benchmark.

    Returns:
        str: The path to the latest benchmark file.

    Raises:
        FileNotFoundError: If no benchmark files are found.
    """
    list_of_files = glob.glob(f'./{benchmark_name}*.json')
    if not list_of_files:
        raise FileNotFoundError(f"No {benchmark_name} files found in the current working directory.")
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file
