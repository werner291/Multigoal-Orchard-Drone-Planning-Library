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
results = load_benchmark_results('benchmark_sphere_scan_euclidean_paths_limited_angle',
                                 'Sphere scan euclidean paths (limited angle) benchmark')

save_to_dir = os.environ.get('FIGURES_DIR', 'generated_figures')
os.makedirs(save_to_dir, exist_ok=True)

num_points = results['num_points']

################################################################
# Plot the distance traveled vs number of points seen for path #
################################################################

# Create a new figure for the plot
plt.figure()

# Loop through each radius in the results and plot the data
for i, per_rad in enumerate(results['equatorial']):
    # Transform the samples into a dataframe
    samples = pd.json_normalize(per_rad['eval']['samples'])

    pct_seen = samples['n_seen'] / num_points * 100.0

    # Plot the distance traveled vs. the number of points seen
    label = "Equatorial Radius" if i == 0 else "_nolegend_"
    plt.plot(samples['distance_traveled'], pct_seen, label=label, color='blue')

# Loop through each radius in the oscillating results and plot the data
for i, per_rad in enumerate(results['oscillating']):
    # Transform the samples into a dataframe
    samples = pd.json_normalize(per_rad['eval']['samples'])

    pct_seen = samples['n_seen'] / num_points * 100.0

    # Plot the distance traveled vs. the number of points seen
    label = "Oscillating Radius" if i == 0 else "_nolegend_"
    plt.plot(samples['distance_traveled'], pct_seen, label=label, color='green')

# Loop through each radius in the spiral results and plot the data
for i, per_rad in enumerate(results['spiral']):
    # Transform the samples into a dataframe
    samples = pd.json_normalize(per_rad['eval']['samples'])

    pct_seen = samples['n_seen'] / num_points * 100.0

    # Plot the distance traveled vs. the number of points seen
    label = "Spiral Radius" if i == 0 else "_nolegend_"
    plt.plot(samples['distance_traveled'], pct_seen, label=label, color='red')

# Add grid lines to the plot
plt.grid()

# Set the title and labels for the axes
plt.title('Sphere Sampling Results')
plt.xlabel('Distance Traveled')
plt.ylabel('Surface Points Seen (%)')

plt.legend()

# Save the plot to a file
plt.savefig(os.path.join(save_to_dir, 'sphere_sampling_distance_vs_points_seen_limited_angle.svg'))
# Display the plot
plt.show()

##############################################
# A plot that isolates the spiral paths only #
##############################################

# Create a new figure for the spiral plot
plt.figure()

# Loop through each radius in the spiral results and plot the data
for i, per_rad in enumerate(results['spiral']):
    # Transform the samples into a dataframe
    samples = pd.json_normalize(per_rad['eval']['samples'])

    pct_seen = samples['n_seen'] / num_points * 100.0

    # Plot the distance traveled vs. the number of points seen
    label = f"Spiral {per_rad['radius_factor']} x {per_rad['turns']}"
    plt.plot(samples['distance_traveled'], pct_seen)

    # Annotate the point with the highest distance traveled with the label as text:
    max_distance = samples['distance_traveled'].max()
    max_points_seen = pct_seen[samples['distance_traveled'].idxmax()]
    plt.text(max_distance, max_points_seen, label)

# Add grid lines to the plot
plt.grid()

# Set the title and labels for the axes
plt.title('Spiral Sampling Results')
plt.xlabel('Distance Traveled')
plt.ylabel('Surface Points Seen (%)')

# Save the plot to a file
plt.savefig(os.path.join(save_to_dir, 'spiral_sampling_distance_vs_points_seen_limited_angle.svg'))
# Display the plot
plt.show()
