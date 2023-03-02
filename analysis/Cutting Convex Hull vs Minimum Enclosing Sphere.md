---
jupyter:
  jupytext:
    formats: ipynb,md
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.14.5
  kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---

```python
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json
```

```python
with open('data/static_sphere_vs_chull.json') as f:
    df = pd.json_normalize(json.load(f))
    
df.columns = df.columns.str.replace("parameters.", '', regex=False)
df.columns = df.columns.str.replace("result.", '', regex=False)
df.columns = df.columns.str.replace("problem.", '', regex=False)

df['time_per_visited'] = df['time']/df['n_visited']
df['length_per_visited'] = df['total_path_length']/df['n_visited']

df
```

```python
df_mean = df.groupby(['planner','nApples'])[['n_visited','time_per_visited','length_per_visited']].agg(['mean','sem']).reset_index()
df_mean
```

```python
plt.figure(figsize=(14,6))

for (i, col) in enumerate(['n_visited','time_per_visited','length_per_visited']):

    plt.subplot(1,3,i+1)
    
    for planner, planner_df in df_mean.groupby('planner'):
    
        x = planner_df['nApples']
        y = planner_df[col,'mean']
        yerr = planner_df[col,'sem']

        plt.errorbar(x,y,yerr=yerr, label=planner)
        plt.title(col.capitalize())
    
plt.legend()
```

```python

```
