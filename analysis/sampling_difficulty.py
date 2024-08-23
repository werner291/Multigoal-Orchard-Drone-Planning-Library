#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.
import os

import matplotlib.pyplot as plt
import pandas as pd
from load_benchmark_results import load_benchmark_results
import seaborn as sns
import numpy as np

# Load the benchmark results
results = load_benchmark_results('benchmark_sampling_difficulty', 'Description of the benchmark')

# Initialize an empty list to store the dataframes
dfs = []

# Iterate over each tree model in the results
for tree_model, data in results.items():
    # Convert the samples dictionary into a dataframe
    df = pd.json_normalize(data['samples'])

    # Add the tree model name, h_radius and z_radius as additional columns
    df['tree_model'] = tree_model

    # Compute the max depth for that tree model:
    # (Actually the min depth, since the depth is negative)
    max_depth = df['signed_depth'].min()

    # Compute the depth ratio for each sample:
    df['depth_ratio'] = df['signed_depth'] / max_depth

    # Append the dataframe to the list
    dfs.append(df)

# Concatenate all dataframes in the list
df = pd.concat(dfs, ignore_index=True)
del dfs

# Plot the depth ratio vs. the percentage colliding, binned by the depth ratio.

# Define the number of bins
n_bins = 10
depth_ratio_bins = pd.cut(df['depth_ratio'], bins=np.linspace(-1, 1, n_bins + 1))

save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

# Plot the data
df.groupby(depth_ratio_bins)['collides'].mean().plot()
plt.grid()
plt.xlabel('Depth (as a ratio of the maximum depth)')
plt.ylabel('Percentage of samples colliding')
plt.savefig(os.path.join(save_to_dir, 'sampling_difficulty.svg'))
plt.show()

# Different one, this one per tree:
df.groupby([depth_ratio_bins,'tree_model'])['collides'].mean().unstack().plot()
plt.grid()
plt.xlabel('Depth (as a ratio of the maximum depth)')
plt.ylabel('Percentage of samples colliding')
plt.savefig(os.path.join(save_to_dir, 'sampling_difficulty_per_tree.svg'))
plt.show()

# And as the Q1,Q2,Q3:
df.groupby([depth_ratio_bins,'tree_model'])['collides'].mean().groupby('depth_ratio').quantile([0.25,0.5,0.75]).unstack().plot()
plt.grid()
plt.xlabel('Depth (as a ratio of the maximum depth)')
plt.ylabel('Percentage of samples colliding')
plt.savefig(os.path.join(save_to_dir, 'sampling_difficulty_per_tree_quantiles.svg'))
plt.show()