#  Copyright (c) 2024 University College Roosevelt
#
#  All rights reserved.


# Import necessary libraries
from load_benchmark_results import load_benchmark_results
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns

import numpy as np
import os
# Plot the boxplot of the time categorized by method index
import matplotlib.ticker as ticker

save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

# Load benchmark results from the specified file
# The file is expected to contain data related to the success rate of goal sampling
results = load_benchmark_results(
    "benchmark_success_by_vector",
    "Comparison of escape success by escape vector compared to arm vector")

df = pd.json_normalize([row for r in results["results"] for row in r if not row['collides']])
df["success"] = ~df["motion_collides"]

sns.lineplot(data=df, x='dot', y='success')
plt.ylim(0, 1)
plt.grid()
plt.title('Success probability by dot product of arm vector and escape vector')
plt.savefig(os.path.join(save_to_dir, 'success_probability_by_armdot.svg'))
plt.show()
