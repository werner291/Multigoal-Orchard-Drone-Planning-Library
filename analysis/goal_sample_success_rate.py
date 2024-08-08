# Import necessary libraries
from load_benchmark_results import load_benchmark_results
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np


# Load benchmark results from the specified file
# The file is expected to contain data related to the success rate of goal sampling
results = load_benchmark_results(
    "benchmark_goal_sample_success_rate",
    "Goal sample success rate graph plotting script.")

# Extract the maximum number of samples from the results
max_samples = results['max_samples']

# Normalize the JSON data from the results into a flat table
runs = pd.json_normalize(results['results'])

# Sort them by increasing depth:
runs = runs.sort_values('goal_depth')

# Calculate the Goal Sample Success Rate (GSSR) as a percentage, by subtracting
# the ratio of collisions to max_samples from 1 and multiplying by 100
runs['gssr'] = (1.0 - runs['collisions'] / max_samples) * 100.0

runs['depth_ratio'] = runs.groupby('tree_model')['goal_depth'].transform(lambda x: x / x.max())
runs['depth_ratio_bin'] = pd.cut(runs['depth_ratio'], bins=5)

# Calculate the bin middles
bins = pd.cut(runs['depth_ratio'], bins=5)
bin_middles = bins.apply(lambda x: (x.right + x.left) / 2).unique()
bin_middles = np.sort(bin_middles)
# Define the list of robot models
robot_models = ['0.5JHV', '0.75JHV', '1JHV']

# Create a figure with 3 subplots, one for each robot model
fig, axs = plt.subplots(1, len(robot_models), figsize=(10, 5))

# Iterate over the robot models and create a subplot for each one
for i, robot_model in enumerate(robot_models):
    # Filter the runs for the specific robot model
    robot_runs = runs[runs['robot_model'] == robot_model]

    # Plot the data
    robot_runs.groupby(['tree_model', 'depth_ratio_bin'])['gssr'].mean().groupby('tree_model').plot(ax=axs[i])

    # Set the x-ticks to the bin middles
    axs[i].set_xticks(bin_middles * 4.0)
    axs[i].set_xticklabels(np.round(bin_middles, 2))

    # Set the labels
    axs[i].set_ylabel('Goal Sample Success Rate (%)')
    axs[i].set_xlabel('Depth Ratio (Depth / Max Depth)')
    axs[i].set_title(f'Robot Model: {robot_model}')

# Save the figure
plt.savefig('goal_sample_success_rate_means.svg')
plt.show()