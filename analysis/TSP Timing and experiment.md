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
import pandas as pd
import json
import matplotlib.pyplot as plt
import numpy as np
```

```python
with open('tsp_test_results.json') as f:
  data = json.load(f)
```

```python
data
```

```python
for run in data:
    
    costs = [run['initial_ordering_cost']] + [update['update_ordering_cost'] for update in run['updates']]
    times = [run['initial_ordering_time']] + [update['update_ordering_time'] for update in run['updates']]
    
    if run['name'] == 'LCI':
        color = 'r'
    else:
        color = 'g'
    
    plt.plot(costs, color=color, label=run['name'])
```

```python
df = []

for run in data:
    
    name = run['name']
    costs = [run['initial_ordering_cost']] + [update['update_ordering_cost'] for update in run['updates']]
    times = [run['initial_ordering_time']] + [update['update_ordering_time'] for update in run['updates']]
    cumtimes = np.cumsum(np.array(times)/1000)
    
    for (cost_i,(cost,time,time_cum)) in enumerate(zip(costs,times,cumtimes)):
        df.append({
            'name': name,
            'i': cost_i,
            'cost': cost,
            'time': time/1000,
            'time_cum': time_cum
        })
    
df = pd.DataFrame(df)
```

```python
df_agg = df.groupby(['name', 'i']).agg(['mean', 'sem']).reset_index()
df_agg.head()
```

```python
plt.figure(figsize=(15,8))

for method, m_df in df_agg.groupby('name'):

    # Cost
    
    plt.subplot(1,2,1)
    
    cost = m_df['cost','mean']
    cost_err = 1.96 * m_df['cost','sem']
    
    plt.plot(m_df['i'], cost, label=method)
    
    plt.fill_between(m_df['i'], cost-cost_err, cost+cost_err, alpha=0.5)

    # Time
    
    plt.subplot(1,2,2)
    
    time = m_df['time','mean']
    time_err = 1.96 * m_df['time','sem']
    
    plt.plot(m_df['i'], time, label=method)
    
    plt.fill_between(m_df['i'], time-time_err, time+time_err, alpha=0.5)

plt.subplot(1,2,1)
plt.grid()
plt.xlabel('Points added')
plt.ylabel('Euclidean length cost')

plt.subplot(1,2,2)
plt.grid()
plt.xlabel('Points added')
plt.ylabel('Iteration time (ms)')
plt.legend()
```

```python

```

```python

```
