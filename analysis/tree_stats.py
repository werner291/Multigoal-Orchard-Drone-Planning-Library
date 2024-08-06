import json
import matplotlib.pyplot as plt
import argparse

from loach_benchmark_results import load_benchmark_results

def main(results):
    # Extract the number of fruits for each tree
    tree_names = []
    n_fruits = []

    for tree_name, stats in results.items():
        tree_names.append(tree_name)
        n_fruits.append(stats['n_fruit'])

    # Plot the bar graph
    plt.figure(figsize=(12, 6))
    bars = plt.bar(tree_names, n_fruits, color='skyblue')
    plt.yscale('log')  # Set the vertical axis to logarithmic
    plt.xticks(rotation=90)  # Rotate the tree names on the horizontal axis

    # Annotate each bar with the number of fruits
    for bar, n_fruit in zip(bars, n_fruits):
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, yval, int(yval), va='bottom', ha='center', fontsize=10)

    # Adding labels and title
    plt.xlabel('Tree Names')
    plt.ylabel('Number of Fruits (log scale)')
    plt.title('Number of Fruits per Tree')

    # Display the plot
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot number of fruits per tree from a JSON file.')
    parser.add_argument('file_path', nargs='?', type=str, help='Path to the JSON file containing tree stats')
    parser.add_argument('--benchmark_name', type=str, default='benchmark_tree_stats', help='Benchmark name prefix for finding the latest file')
    args = parser.parse_args()

    if args.file_path:
        with open(args.file_path, 'r') as file:
            data = json.load(file)
        results = data['results']
        print(f"Using file: {args.file_path}")
    else:
        results = load_benchmark_results(args.benchmark_name)

    main(results)
