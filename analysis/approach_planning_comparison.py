#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.

# Import necessary libraries
from load_benchmark_results import load_benchmark_results
import pandas as pd
import matplotlib as mpl

# Circumvent what appears to be a bug in Pycharm/CLion:
# https://youtrack.jetbrains.com/issue/PY-75269/Error-after-updating-to-latest-PyCharm-2024.2-CE-Failed-to-enable-GUI-event-loop-integration-for-qt
mpl.use("Qt5Agg")
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

df = pd.json_normalize(results['results'])

##########################################
# Distribution of timings of the methods #
##########################################

plt.figure(figsize=(10, 6))
sns.violinplot(x='method', y='time_ms', data=df)
# plt.yscale('log')
plt.title('Violin Plot of Time (ms) by Method Index')
plt.xlabel('Method Index')
plt.ylabel('Time (ms)')
plt.grid()

# Use ScalarFormatter to write out the numbers on the y-axis
formatter = ticker.ScalarFormatter(useMathText=False)
formatter.set_scientific(False)
plt.gca().yaxis.set_major_formatter(formatter)

# Save the plot
plt.savefig(os.path.join(save_to_dir, 'approach_planning_comparison_time_violin.svg'))

plt.show()
