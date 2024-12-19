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
results = load_benchmark_results('benchmark_sphere_sampling', 'Sphere sampling benchmark')

save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

num_points = results['num_points']

#######################################################################
# Plot the distance traveled vs number of points seen for each radius #
#######################################################################

# Create a new figure for the plot
plt.figure()

# Loop through each radius in the results and plot the data
for per_rad in results['per_radius']:
    # Transform the samples into a dataframe
    samples = pd.json_normalize(per_rad['samples'])

    pct_seen = samples['n_seen'] / num_points * 100.0

    # Plot the distance traveled vs. the number of points seen
    plt.plot(samples['distance_traveled'], pct_seen, label=f"Radius: {per_rad['radius']}")

# Add grid lines to the plot
plt.grid()

# Set the title and labels for the axes
plt.title('Sphere Sampling Results')
plt.xlabel('Distance Traveled')
plt.ylabel('Surface Points Seen (%)')

# Save the plot to a file
plt.savefig(os.path.join(save_to_dir, 'sphere_sampling_distance_vs_points_seen.svg'))
# Display the plot
plt.show()
