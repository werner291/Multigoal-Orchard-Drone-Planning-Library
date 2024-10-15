#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.

# Import necessary libraries
from load_benchmark_results import load_benchmark_results
import pandas as pd
import matplotlib as mpl

# Circumvent what appears to be a bug in Pycharm/CLion:
# https://youtrack.jetbrains.com/issue/PY-75269/Error-after-updating-to-latest-PyCharm-2024.2-CE-Failed-to-enable-GUI-event-loop-integration-for-qt
# mpl.use("Qt5Agg")
import matplotlib.pyplot as plt
import seaborn as sns

import numpy as np
import os
# Plot the boxplot of the time categorized by method index
import matplotlib.ticker as ticker

# Load benchmark results from the specified file
# The file is expected to contain data related to the success rate of goal sampling
results = load_benchmark_results(
    "benchmark_approach_planning_comparison",
    "Comparison of different approach planning methods.")

save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

df_problems = pd.json_normalize(results['problems'])
methods = results['methods']

# Flatten the "annotations" column:
for r in results['results']:
    if r['annotations'] is None:
        continue
    for i, annotation in r.pop('annotations').items():
        r[i] = annotation

df = pd.json_normalize(results['results'])
df = df.join(df_problems, on='problem')
df['method'] = df['method'].replace({i: method for i, method in enumerate(methods)})

# Calculate time_per_target
df['time_per_target'] = df['time_ms'] / df['n_targets']
df['success_rate'] = df['targets_reached'] / df['n_targets']
df['motions_checked_per_target'] = df['motions_checked'] / df['n_targets']

# Calculate the mean path length and mean path distance
df['mean_path_n_waypt'] = df['path_lengths'] \
    .apply(lambda x: np.mean([length for length in x if length is not None]))
df['mean_path_distance'] = df['path_distances'] \
    .apply(lambda x: np.mean([length for length in x if length is not None]))

# There are so many rrt variants that we'd like to group them together
df['method_category'] = df['method'].apply(lambda x: 'rrt' if 'rrt' in x.lower() else x)

##########################################
# Distribution of timings of the methods #
##########################################

plt.figure(figsize=(10, 6))
# We exclude straight_in because it's pretty much O(1) time.
sns.kdeplot(df[df['method'] != 'straight_in'], x='time_per_target', hue='method').set(xlim=0)
plt.title('Plot of Time (ms) by Method')
plt.grid()

# Save the plot
plt.savefig(os.path.join(save_to_dir, 'time_per_target_by_method.svg'))
plt.show()

############################################################
# Distribution of timings of the methods (boxplot by tree) #
############################################################

plt.figure(figsize=(10, 6))
sns.barplot(hue='method', y='time_per_target', x='problem', data=df)

plt.title('Box Plot of Time (ms) by Method Index Grouped by Tree')
plt.xlabel('Tree index')
plt.ylabel('Time (ms)')
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_avg_time_by_tree.svg'))
plt.show()

############################################################
# Distribution of timings of the methods (median of means) #
############################################################

plt.figure(figsize=(10, 6))
sns.barplot(y='time_per_target', x='method',
            data=df.groupby(['method', 'problem'])['time_per_target'].mean().reset_index())
# Rotate the x-axis labels
plt.xticks(rotation=90)
plt.tight_layout()
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_avg_time_by_method.svg'))
plt.show()

#################################################################
# Distribution of success rate of the methods (boxplot by tree) #
#################################################################

plt.figure(figsize=(10, 6))
sns.barplot(hue='method', y='success_rate', x='problem', data=df)

plt.title('Box Plot of % reached by Method Index Grouped by Tree')
plt.xlabel('Method Index')
plt.ylabel('Success rate (0 to 1)')
plt.ylim(0, 1)
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_success_rate_by_tree.svg'))
plt.show()

##########################################################
# Distribution of success rate of the straight_in method #
##########################################################

plt.figure(figsize=(10, 6))
sns.barplot(hue='method', y='success_rate', x='problem', data=df[df['method'] == 'straight_in'])

plt.title('Box Plot of % reached by straight_in Method Grouped by Tree')
plt.xlabel('Method Index')
plt.ylabel('Success rate (0 to 1)')
plt.ylim(0, 1)
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_success_rate_straight_in_by_tree.svg'))
plt.show()

#################################################################
# Distribution of success rate of the methods (median of means) #
#################################################################

plt.figure(figsize=(10, 6))
sns.barplot(y='success_rate', x='method',
            data=df.groupby(['method', 'problem'])['success_rate'].mean().reset_index())
# Rotate the x-axis labels
plt.xticks(rotation=90)
plt.tight_layout()
plt.grid()
plt.title('Success rate by method')

plt.savefig(os.path.join(save_to_dir, 'approach_planning_success_rate_by_method.svg'))
plt.show()

################################################################
# A scatter plot of the success rate vs the average time taken #
################################################################

plt.figure(figsize=(10, 6))
sns.scatterplot(data=df, y='success_rate', x='time_per_target', hue='method')

plt.title('Plot of success rate vs time spent per target')
plt.xlabel('Time (ms)')
plt.ylabel('Success rate')
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_success_rate_by_time.svg'))
plt.show()

###############################
# Path lengths of the methods #
###############################

plt.figure(figsize=(10, 6))
sns.barplot(y='mean_path_n_waypt', x='method',
            data=df.groupby(['method', 'problem'])['mean_path_n_waypt'].mean().reset_index())
plt.title('Mean number of waypoints in path by method')
plt.xticks(rotation=90)
plt.tight_layout()
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_path_length_by_method.svg'))
plt.show()

##################################
# Path distances of the methods #
##################################

plt.figure(figsize=(10, 6))
sns.barplot(y='mean_path_distance', x='method',
            data=df.groupby(['method', 'problem'])['mean_path_distance'].mean().reset_index())
plt.title('Mean path distance by method')
plt.xticks(rotation=90)
plt.tight_layout()
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_path_distance_by_method.svg'))
plt.show()

#####################################
# Cost in terms of motions checked. #
#####################################

plt.figure(figsize=(10, 6))
sns.boxplot(y='motions_checked_per_target', x='method', data=df[df['method'] != 'straight_in'])
plt.title('Mean number of motions checked by method')
plt.xticks(rotation=90)
plt.ylim(0, 100)
plt.tight_layout()
plt.grid()

plt.savefig(os.path.join(save_to_dir, 'approach_planning_motions_checked_by_method.svg'))
plt.show()

#############################################################################
# Cost in terms of motions checked per time, by method, by tree complexity. #
#############################################################################

fig, axes = plt.subplots(2, 1, figsize=(10, 12))

# First subplot: Time per target by method and tree complexity
sns.lineplot(ax=axes[0], x='n_targets', y='time_per_target', hue='method', style='method_category', data=df)
axes[0].set_xscale('log')
axes[0].set_ylim(0, 20)
axes[0].set_title('Time per target by method and tree complexity')
axes[0].grid()
axes[0].set_ylabel('Time per target (ms)')
axes[0].set_xlabel('Number of targets')

# Second subplot: Time per target by method and triangle count
sns.lineplot(ax=axes[1], x='n_trunk_triangles', y='time_per_target', hue='method', style='method_category', data=df)
axes[1].set_xscale('log')
axes[1].set_ylim(0, 20)
axes[1].set_title('Time per target by method and triangle count')
axes[1].grid()
axes[1].set_ylabel('Time per target (ms)')
axes[1].set_xlabel('Triangle count')

plt.tight_layout()
plt.savefig(os.path.join(save_to_dir, 'approach_planning_time_per_target_by_tree_complexity.svg'))
plt.show()

#####################################################################################
# Success rate in terms of motions checked per time, by method, by tree complexity. #
####################################################################################

fig, axes = plt.subplots(2, 1, figsize=(10, 12))

# First subplot: Time per target by method and tree complexity
sns.lineplot(ax=axes[0], x='n_targets', y='success_rate', hue='method', style='method_category', data=df)
axes[0].set_xscale('log')
axes[0].set_title('Success rate by method and tree complexity')
axes[0].grid()
axes[0].set_ylabel('Success rate')
axes[0].set_xlabel('Number of targets')

# Second subplot: Time per target by method and triangle count
sns.lineplot(ax=axes[1], x='n_trunk_triangles', y='success_rate', hue='method', style='method_category', data=df)
axes[1].set_xscale('log')
axes[1].set_title('Success rate by method and triangle count')
axes[1].grid()
axes[1].set_ylabel('Success rate')
axes[1].set_xlabel('Triangle count')

plt.tight_layout()
plt.savefig(os.path.join(save_to_dir, 'approach_planning_success_rate_by_tree_complexity.svg'))
plt.show()

#############################################################################
# Cost in terms of motions checked per time, by method, by tree complexity. #
#############################################################################

fig, axes = plt.subplots(2, 1, figsize=(10, 18))

# First subplot: Time per target by method and tree complexity
sns.lineplot(ax=axes[0], x='n_targets', y='time_per_target', hue='method', style='method_category', data=df)
axes[0].set_xscale('log')
axes[0].set_ylim(0, 20)
axes[0].set_title('Time per target by method and tree complexity')
axes[0].grid()
axes[0].set_ylabel('Time per target (ms)')
axes[0].set_xlabel('Number of targets')

# Second subplot: Time per target by method and triangle count
sns.lineplot(ax=axes[1], x='n_trunk_triangles', y='time_per_target', hue='method', style='method_category', data=df)
axes[1].set_xscale('log')
axes[1].set_ylim(0, 20)
axes[1].set_title('Time per target by method and triangle count')
axes[1].grid()
axes[1].set_ylabel('Time per target (ms)')
axes[1].set_xlabel('Triangle count')

plt.tight_layout()
plt.savefig(
    os.path.join(save_to_dir, 'approach_planning_time_vs_tree_complexity.svg'))
plt.show()

#####################################################################################
# Success rate in terms of motions checked per time, by method, by tree complexity. #
#####################################################################################

# Create the figure and subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

# Plot the number of targets
sns.barplot(data=df_problems, x=df_problems.index, y='n_targets', ax=ax1, color='tab:blue')
ax1.set_xlabel('Tree Model')
ax1.set_ylabel('Number of Targets')
ax1.set_title('Number of Targets per Tree Model')
ax1.grid()

# Plot the number of triangles
sns.barplot(data=df_problems, x=df_problems.index, y='n_trunk_triangles', ax=ax2, color='tab:orange')
ax2.set_xlabel('Tree Model')
ax2.set_ylabel('Number of Triangles')
ax2.set_title('Number of Triangles per Tree Model')
ax2.grid()

# Set the title for the entire figure
fig.suptitle('Number of Targets and Triangles per Tree Model', fontsize=16)

# Save and show the plot
fig.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(os.path.join(save_to_dir, 'targets_and_triangles_per_tree_model.svg'))
plt.show()
