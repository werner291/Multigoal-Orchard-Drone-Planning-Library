#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.

import os
import json
import matplotlib.pyplot as plt
import pandas as pd
from load_benchmark_results import load_benchmark_results

################################################
# Load the benchmark results and process them. #
################################################

results = load_benchmark_results('benchmark_tree_complexity_metrics', 'Tree complexity metrics')
save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

# Convert the results into a list of dataframes
dataframes = {}
for i, (key, value) in enumerate(results.items()):
    print(key)  # Print the key for progress tracking

    # Parse into a DataFrame
    df = pd.DataFrame(value)

    # Sort by cell radius so that we can plot the cumulative volume
    df.sort_values(by='cell_radius', inplace=True)
    df['cum_volume'] = df['cell_volume'].cumsum()
    df['cum_volume'] /= df['cum_volume'].max()
    df['tree'] = i
    dataframes[key] = df

##########################################################################################
# Plot the cell radius vs. cumulative volume, with a different line for each tree model. #
##########################################################################################

fig, ax = plt.subplots(figsize=(10, 6))

plt.yscale('log')

for key, df in dataframes.items():
    df.plot(x='cum_volume', y='cell_radius', ax=ax)

ax.set_xlabel('Cumulative Volume')
ax.set_ylabel('Cell Radius')
ax.set_title('Cell Radius vs. Cumulative Volume by Tree Model')
ax.grid()

plt.tight_layout()

plt.savefig(os.path.join(save_to_dir, 'cell_radius_vs_cumulative_volume.svg'))
plt.show()

##########################################################################################
# Grab the cell size at 50% of the cumulative volume:
##########################################################################################

cell_radius_at_50_percent = {}
for key, df in dataframes.items():
    cell_radius_at_50_percent[key] = df[df['cum_volume'] > 0.5]['cell_radius'].iloc[0]

# Dump it as JSON:
with open(os.path.join(save_to_dir, 'cell_radius_at_50_percent.json'), 'w') as file:
    json.dump(cell_radius_at_50_percent, file)

# Barplot per tree model:
fig, ax = plt.subplots(figsize=(10, 6))
pd.Series(cell_radius_at_50_percent).plot(kind='bar', ax=ax)
ax.set_xlabel('Tree Model')
ax.set_ylabel('Cell Radius at 50% of Cumulative Volume')
ax.set_title('Cell Radius at 50% of Cumulative Volume by Tree Model')
ax.grid()

plt.tight_layout()

plt.savefig(os.path.join(save_to_dir, 'cell_radius_at_50_percent.svg'))

plt.show()
