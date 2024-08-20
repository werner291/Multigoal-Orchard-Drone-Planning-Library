# Import necessary libraries
from load_benchmark_results import load_benchmark_results
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import os

# Load benchmark results from the specified file
# The file is expected to contain data related to the success rate of goal sampling
results = load_benchmark_results(
    "benchmark_goal_sample_success_rate",
    "Goal sample success rate graph plotting script.")

save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

# Extract the maximum number of samples from the results
max_samples = results['max_samples']

# Normalize the JSON data from the results into a flat table
runs = pd.json_normalize(results['results'])

# Drop any rows where the robot model ends in J: (those are the fixed-stick ones that don't work)
runs = runs[~runs['robot_model'].str.endswith('J')]

# Sort them by increasing depth:
runs = runs.sort_values('goal_depth')

# Calculate the Goal Sample Success Rate (GSSR) as a percentage, by subtracting
# the ratio of collisions to max_samples from 1 and multiplying by 100
runs['gssr'] = (1.0 - runs['collisions'] / max_samples) * 100.0

runs['depth_ratio'] = runs.groupby('tree_model')['goal_depth'].transform(lambda x: x / x.max())
runs['depth_ratio_bin'] = pd.cut(runs['depth_ratio'], bins=5)

# A flag about whether any of the samples were successful at all
runs['any_success'] = runs['collisions'] < max_samples

# Calculate the bin middles
bins = pd.cut(runs['depth_ratio'], bins=5)
bin_middles = bins.apply(lambda x: (x.right + x.left) / 2).unique()
bin_middles = np.sort(bin_middles)

# Define the list of robot models
robot_models = ['0.5JHV', '0.75JHV', '1JHV']

#############################################
# First, plot every tree as a separate line #
#############################################

# Create a figure with 3 subplots, one for each robot model
fig, axs = plt.subplots(1, len(robot_models), figsize=(10, 5), sharey=True)

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
    axs[i].grid()

plt.suptitle('Goal Sample Success Rate Means by Tree Model')

# Save the figure
plt.savefig(os.path.join(save_to_dir, 'goal_sample_success_rate_means.svg'))
plt.show()

###############################
# Then, as a combined average #
###############################

# Create a figure with 3 subplots, one for each robot model
fig, axs = plt.subplots(1, len(robot_models), figsize=(10, 5), sharey=True)

# Iterate over the robot models and create a subplot for each one
for i, robot_model in enumerate(robot_models):
    # Filter the runs for the specific robot model
    robot_runs = runs[runs['robot_model'] == robot_model]

    # Compute median and interquartile range for each depth ratio bin
    robot_runs.groupby('depth_ratio_bin')['gssr'].median().plot(ax=axs[i])
    robot_runs.groupby('depth_ratio_bin')['gssr'].quantile(0.25).plot(ax=axs[i], linestyle='--')
    robot_runs.groupby('depth_ratio_bin')['gssr'].quantile(0.75).plot(ax=axs[i], linestyle='--')

    # Set the x-ticks to the bin middles
    axs[i].set_xticks(bin_middles * 4.0)
    axs[i].set_xticklabels(np.round(bin_middles, 2))

    # Set the labels
    axs[i].set_ylabel('Goal Sample Success Rate (%)')
    axs[i].set_xlabel('Depth Ratio (Depth / Max Depth)')
    axs[i].set_title(f'Robot Model: {robot_model}')

    axs[i].grid()

plt.suptitle('Goal Sample Success Rate Medians')

# Save the figure
plt.savefig(os.path.join(save_to_dir, 'goal_sample_success_rate_total_mean.svg'))
plt.show()

########################################################################################
# Then by the robot model; specifically, we'd like to try different numbers of joints. #
########################################################################################

# Let's create a combined plot, with the depth bin on the horizontal axis, and the GSSR on the vertical axis.
# Each line shall be the median GSSR for a specific robot model.

# Create a figure
arm_lengths = ['0.5', '0.75', '1']
joint_configs = ['JH', 'JV', 'JHH', 'JVV', 'JHV']

fig, ax = plt.subplots(1, len(arm_lengths), figsize=(10, 5), sharey=True, sharex=True)

# Get a color palette:
color_names = ['blue', 'orange', 'green', 'red', 'purple']

for i, length in enumerate(arm_lengths):

    for j, joint_config in enumerate(joint_configs):
        robot_model = f'{length}{joint_config}'
        robot_runs = runs[runs['robot_model'] == robot_model]
        robot_runs.groupby('depth_ratio_bin')['gssr'].median().plot(label=robot_model, ax=ax[i])

    ax[i].set_xticks(bin_middles)
    ax[i].set_xticklabels(np.round(bin_middles, 2))
    ax[i].grid()
    ax[i].legend()

plt.suptitle('Goal Sample Success Rate Medians by Robot Model')
plt.savefig(os.path.join(save_to_dir, 'goal_sample_success_rate_robot_model.svg'))

plt.show()

###########################################################################
# Then by the robot model, we'd like to create a bar chart for the number #
# of goals for which at least one sample was successful.                  #
###########################################################################

# Create a figure
fig = plt.figure(figsize=(10, 5))

# Compute the number of goals for which at least one sample was successful
(runs.groupby('robot_model')['any_success'].mean() * 100.0).plot(kind='bar')

# Add a title
plt.title('Percent of Goals with at least one successful sample')

# Add labels
plt.xlabel('Robot Model')
plt.ylabel('Percent of Goals')

# Add a grid
plt.grid()

# Save the figure
plt.savefig(os.path.join(save_to_dir, 'minimum_one_successful_goal_sample_by_robot_model.svg'))

# Display the figure
plt.show()