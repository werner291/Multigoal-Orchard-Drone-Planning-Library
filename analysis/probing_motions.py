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
