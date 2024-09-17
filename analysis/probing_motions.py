#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.

# Import necessary libraries
from load_benchmark_results import load_benchmark_results
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl

# Circumvent what appears to be a bug in Pycharm/CLion:
# https://youtrack.jetbrains.com/issue/PY-75269/Error-after-updating-to-latest-PyCharm-2024.2-CE-Failed-to-enable-GUI-event-loop-integration-for-qt
mpl.use("Qt5Agg")

import numpy as np
import os

# Load benchmark results from the specified file
# The file is expected to contain data related to the success rate of goal sampling
results = load_benchmark_results(
    "benchmark_probing_motions",
    "Success rate of straight-arm probing motions.")

max_samples = results['max_samples']

save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

df = pd.json_normalize(results['results'])
df['collision_free_samples'] = max_samples - df['collisions']

df['depth_ratio'] = df['signed_goal_depth'] / df.groupby('tree_model')['signed_goal_depth'].transform('max')
# sort by increasing depth
df = df.sort_values('depth_ratio')

depth_bins = pd.cut(df['depth_ratio'], bins=np.linspace(-1, 1, 11))

stats = df.groupby([depth_bins, 'tree_model'])[['collisions', 'successful_pullouts']] \
    .mean().groupby('depth_ratio').quantile([0.25, 0.5, 0.75]) \
    .unstack()
ax = (max_samples - stats['collisions']).plot(color='blue')
stats['successful_pullouts'].plot(color='green', ax=ax)
plt.grid()
plt.title('Probing motions and pullout mean success rate by depth')
plt.legend(['Collisions', 'Successful pullouts'])
plt.savefig(os.path.join(save_to_dir, 'probing_motions.svg'))
plt.show()

# Let's do the conditional success rates: P(successful pullout | non-collision)
df['conditional_pullout_success'] = df['successful_pullouts'] / (max_samples - df['collisions'])

df.groupby([depth_bins, 'tree_model'])['conditional_pullout_success'] \
    .mean().groupby('depth_ratio').quantile([0.25, 0.5, 0.75]) \
    .unstack().plot()
plt.grid()
plt.title('Probing motions conditional pullout success rate by depth')
plt.legend(['25th percentile', '50th percentile', '75th percentile'])
plt.savefig(os.path.join(save_to_dir, 'probing_motions_conditional.svg'))
plt.show()

# Now collectively: P(at least one successful pullout | at least one non-collision)
df['any_pullout_success'] = df['successful_pullouts'] > 0
df['any_non_collision'] = df['collisions'] < max_samples
df['any_pullout_success_conditional'] = df['any_pullout_success'].replace({True: 1, False: 0}) / df[
    'any_non_collision'].replace({True: 1, False: 0})

df.groupby([depth_bins, 'tree_model'])['any_pullout_success_conditional'] \
    .mean().groupby('depth_ratio').quantile([0.25, 0.5, 0.75]) \
    .unstack().plot()
plt.grid()
plt.title('Probing motions conditional any pullout success rate by depth')
plt.legend(['25th percentile', '50th percentile', '75th percentile'])
plt.savefig(os.path.join(save_to_dir, 'probing_motions_any_conditional.svg'))
plt.show()

# We would like to know: if we have a collision-free goal sample, but the pullout fails, how likely is RRT to succeed?
#
# That is, we'd like to know P(RRT succeeds | (goals sample available AND pullout fails))
# First, just as a single number:
df['conditional_rrt_success'] = df['successful_conditional_rrts'] / (
        max_samples - df['collisions'] - df['successful_pullouts'])
print(f'P(RRT succeeds | (goal sample available AND pullout fails)) = {df["conditional_rrt_success"].mean()}')

# The same, normalize by the tree:
conditional_by_tree = df.groupby(['tree_model'])['conditional_rrt_success']
by_tree_mean = conditional_by_tree.mean()
print(f'P(RRT succeeds | (goal sample available AND pullout fails)) by tree:')
print('Mean:', by_tree_mean.mean())
print('Q1:', by_tree_mean.quantile(0.25))
print('Median:', by_tree_mean.median())
print('Q3:', by_tree_mean.quantile(0.75))

# Per tree, a bar plot:
by_tree_mean.plot(kind='bar')
plt.title('RRT success rate after failed pullout')
plt.ylabel('P(RRT succeeds | (goal sample available AND pullout fails))')
plt.grid()
plt.savefig(os.path.join(save_to_dir, 'probing_motions_rrt_success.svg'))
plt.show()

# Alternatively, we'd like to see how much this improves our performance per tree, as a stacked bar chart, where the conditional improvement is stacked on top of the base success rate.
#
# Essentially: P(RRT succeeds OR pullout succeeds | goal sample available)

df['any_success'] = (df['successful_conditional_rrts'] > 0) | (df['successful_pullouts'] > 0)
print(f'P(RRT succeeds OR pullout succeeds | goal sample available) = {df["any_success"].mean()}')

# The same, normalize by the tree:
any_by_tree = df.groupby(['tree_model'])['any_success']
by_tree_mean = any_by_tree.mean()
print(f'P(RRT succeeds OR pullout succeeds | goal sample available) by tree:')
print('Mean:', by_tree_mean.mean())
print('Q1:', by_tree_mean.quantile(0.25))
print('Median:', by_tree_mean.median())
print('Q3:', by_tree_mean.quantile(0.75))

# Per tree, a stackedbar plot, showing the contribution from the RRT fallback separately:

df['successful_pullouts_conditional'] = df['successful_pullouts'] / df['collision_free_samples']
df['successful_rrts_conditional'] = df['successful_conditional_rrts'] / df['collision_free_samples']

df.groupby(['tree_model'])[['successful_pullouts_conditional', 'successful_rrts_conditional']].mean().plot(kind='bar',
                                                                                                           stacked=True)
plt.title('RRT success rate after failed pullout')
plt.ylabel('P(RRT succeeds OR pullout succeeds | goal sample available)')
plt.grid()
plt.savefig(os.path.join(save_to_dir, 'probing_motions_rrt_success_stacked.svg'))
plt.show()

df['any_sample'] = df['collision_free_samples'] > 0
df['any_pullout'] = df['successful_pullouts'] > 0
df['any_fallback_rrt'] = df['successful_rrts_conditional'] > 0
df['any_pullout_with_fallback'] = df['any_pullout'] | df['any_fallback_rrt']
df['any_main_rrt'] = df['successful_rrts'] > 0

# Now, group these together by tree model, take the mean, and plot:
any_by_tree_rrt = df[df['any_sample']].groupby(['tree_model'])[
    ['any_pullout', 'any_pullout_with_fallback', 'any_main_rrt']].mean()
any_by_tree_rrt.plot(kind='bar', ylim=(0.9, 1))
plt.title('P(any success) for various operations, per tree')
plt.ylabel('P(any success)')
plt.grid()
plt.savefig(os.path.join(save_to_dir, 'probing_motions_any_success.svg'))
plt.show()

# Plot the mean of means:
any_by_tree_rrt.mean().plot(kind='bar', ylim=(0.9, 1))
plt.title('P(any success) for various operations, mean of trees')
plt.grid()
plt.tight_layout()
plt.savefig(os.path.join(save_to_dir, 'probing_motions_any_success_mean_of_trees.svg'))
plt.show()

# Get some plots on the costs too:
df.groupby(['tree_model', 'goal_index']).mean().plot.scatter(x='pullout_attempts', y='rrt_checked_motions', alpha=0.5)
plt.title('RRT checked motions vs pullout attempts')
plt.yscale('log')
plt.grid()
plt.tight_layout
plt.savefig(os.path.join(save_to_dir, 'probing_motions_rrt_checked_vs_pullout_attempts.png'))
plt.show()
