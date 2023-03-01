---
jupyter:
jupytext:
formats: ipynb,md
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
df.columns = df.columns.str.replace("problem.", '', regex=False)

param_cols = ['n', 'n_discoverable', 'n_given', 'planner', 'visibility_model']

df['time_per_visited'] = df['time'] / df['n_visited']
df['length_per_visited'] = df['total_path_length'] / df['n_visited']
df['n'] = df['n_discoverable'] + df['n_given']
df['visited_any'] = df['n_visited'] > 0

df = df[df['n_given'] > 0]

df.head()
```

```python
df_mean = df.groupby(param_cols).agg(['mean', 'sem']).reset_index()
df_mean
```

```python
plt.figure(figsize=(10,20))

metrics = ['n_visited', 'time_per_visited','length_per_visited']

n_cols = len(metrics)
n_rows = len(df_mean.groupby(['visibility_model','n']))

for col_i, column in enumerate(metrics):

    for (row_i,(n, n_df)) in enumerate(df_mean.groupby(['visibility_model','n'])):
        
        plt.subplot(n_rows, n_cols, col_i + n_cols * row_i + 1)

        for (planner, planner_df) in n_df.groupby('planner'):
                        
            plt.errorbar(planner_df['n_discoverable'], 
                         planner_df[column,'mean'], 
                         yerr=(1.96*planner_df[column,'sem']),
                         capsize=4,
                         label=planner)

        plt.ylim(bottom=0)
            
        plt.legend()
        plt.xlabel('N Discoverable')
        plt.ylabel(column)
        plt.title('N = {}\n({})'.format(n, column))
        plt.grid()
        
plt.tight_layout()

plt.show()
```

```python
df_mean[df_mean['visited_any']['mean'] < 1][param_cols + ['visited_any']]
```

```python

```
