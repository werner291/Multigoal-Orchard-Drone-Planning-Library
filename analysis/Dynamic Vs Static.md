---
jupyter:
jupytext:
text_representation:
extension: .md
format_name: markdown
format_version: '1.3'
jupytext_version: 1.14.4
kernelspec:
display_name: Python 3 (ipykernel)
language: python
name: python3
---

```python
import numpy as np
import json
import pandas as pd
import matplotlib.pyplot as plt
```

```python
with open('data/dynamic_log.json') as f:
  df = pd.json_normalize(json.load(f))

df.columns = df.columns.str.replace("parameters.", '', regex=False)
df.columns = df.columns.str.replace("result.", '', regex=False)

df['time_per_visited'] = df['time'] / df['n_visited']
df['length_per_visited'] = df['total_path_length'] / df['n_visited']
df['n'] = df['n_discoverable'] + df['n_given']

df = df[df['n_given'] > 0]

df.head()
```

```python
df_mean = df.groupby(['n','n_discoverable','n_given','planner']).agg(['mean','sem']).reset_index()
df_mean
```

```python
df_mean
```

```python
def plot_per_n(column):

    for (n, n_df) in df_mean.groupby('n'):
        plt.figure()

        for (planner, planner_df) in n_df.groupby('planner'):
                        
            plt.errorbar(planner_df['n_discoverable'], planner_df[column,'mean'], yerr=(1.96*planner_df[column,'sem']), label='planner')

        plt.legend()
        plt.xlabel('N Discoverable')
        plt.ylabel(column)
        plt.title('N = {} ({})'.format(n, column))
        plt.grid()
        plt.show()
```

```python
plot_per_n('n_visited')
```

```python
plot_per_n('time_per_visited')
```

```python
plot_per_n('length_per_visited')
```

```python

```
