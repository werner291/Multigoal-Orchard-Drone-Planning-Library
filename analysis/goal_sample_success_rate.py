#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.

from load_benchmark_results import load_benchmark_results
import pandas as pd
import seaborn as sns
import matplotlib as plt

results = load_benchmark_results(
    "benchmark_goal_sample_success_rate",
    "Goal sample success rate graph plotting script.")

runs = pd.json_normalize(results['results'])

one_robot = runs[runs['robot_model'] == '0.75JHV']

sns.lineplot(one_robot, x='goal_index', y='collisions', hue='tree_model')