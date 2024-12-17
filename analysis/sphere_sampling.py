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

plt.figure()

# Transform it into a dataframe:
for per_rad in results['per_radius']:
    samples = pd.json_normalize(per_rad['samples'])

    plt.plot(samples['distance_traveled'], samples['n_seen'], label=f"Radius: {per_rad['radius']}")

plt.grid()
plt.legend()
plt.show()
