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
with open('greatcircle_actual.json') as f:
  data = json.load(f)

df = pd.json_normalize(data)

df.head()
```

```python
plt.figure(figsize=(10, 5))

column_names = [
  'shell_path_length',
  'euclidean_path_length']

for i, col in enumerate(column_names):
  x = df[col]
  y = df['actual_length']

  coef = np.polyfit(x, y, 1)
  poly1d_fn = np.poly1d(coef)

  plt.subplot(1, 2, i + 1)
  plt.scatter(x, y, s=20)
  plt.plot([x.min(), x.max()], [poly1d_fn(x.min()), poly1d_fn(x.max())], color='g')

  plt.xlabel(col.capitalize())
  plt.ylabel('Optimized Goal-to-Goal Path Length')
  plt.grid()
```

```python
df.corr()
```
