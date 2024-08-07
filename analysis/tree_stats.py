import json
import matplotlib.pyplot as plt
import argparse

from load_benchmark_results import load_benchmark_results

results = load_benchmark_results('benchmark_tree_stats', 'Plot number of fruits per tree from a JSON file.')

# Define the metrics to plot
metrics = ['n_fruit', 'leaf_triangles', 'trunk_triangles']

# Create subplots
fig, axes = plt.subplots(len(metrics), 1, figsize=(12, 6 * len(metrics)))

# Plot each metric in a separate subplot
for ax, metric in zip(axes, metrics):
    tree_names = []
    values = []

    for tree_name, stats in results.items():
        tree_names.append(tree_name)
        values.append(stats[metric])

    bars = ax.bar(tree_names, values, color='skyblue')
    ax.set_yscale('log')  # Set the vertical axis to logarithmic
    ax.set_xticks(range(len(tree_names)))
    ax.set_xticklabels(tree_names, rotation=90)  # Rotate the tree names on the horizontal axis

    # Annotate each bar with the value
    for bar, value in zip(bars, values):
        yval = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, yval, int(yval), va='bottom', ha='center', fontsize=10)

    # Adding labels and title
    ax.set_ylabel(f'{metric.replace("_", " ").title()} (log scale)')
    ax.set_title(f'{metric.replace("_", " ").title()} per Tree')

# Adjust layout
plt.tight_layout()
plt.subplots_adjust(hspace=1.0, bottom=0.2)
# plt.savefig('tree_stats.svg')
plt.show()
