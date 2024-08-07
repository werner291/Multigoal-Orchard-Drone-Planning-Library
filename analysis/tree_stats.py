import json
import matplotlib.pyplot as plt
import argparse

from loach_benchmark_results import load_benchmark_results

parser = argparse.ArgumentParser(description='Plot number of fruits per tree from a JSON file.')
parser.add_argument('file_path', nargs='?', type=str, help='Path to the JSON file containing tree stats')
parser.add_argument('--benchmark_name', type=str, default='benchmark_tree_stats',
                    help='Benchmark name prefix for finding the latest file')
args = parser.parse_args()

if args.file_path:
    with open(args.file_path, 'r') as file:
        data = json.load(file)
    results = data['results']
    print(f"Using file: {args.file_path}")
else:
    results = load_benchmark_results(args.benchmark_name)

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
plt.subplots_adjust(hspace=0.5, bottom=0.2)
plt.savefig('tree_stats.svg')
plt.show()
