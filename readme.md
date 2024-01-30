# Multigoal Agricultural Drone Planning Library (mgodpl)

## Description

Multigoal Drone Planning Library (mgodpl) is a project developed to facilitate multi-goal drone planning in orchard-like environments.

## Publications

W. Kroneman, J. Valente and A. F. Van Der Stappen, "A fast two-stage approach for multi-goal path planning in a fruit tree," 2023 IEEE International Conference on Robotics and Automation (ICRA), London, United Kingdom, 2023, pp. 1586-1593, doi: 10.1109/ICRA48891.2023.10160281. (https://ieeexplore.ieee.org/document/10160281)

## File structure:

- `src/`: Source code
  - `src/math/`: General-purpose math functions.
  - `src/planning/`: Path-planning algorithms and related utilities.
  - `src/visualization/`: Visualization utilities.
  - `src/visualizations/`: Various vizualizations, primarily focused on presentations and interaction.
  - `src/experiments/`: Code for running experiments with the goal of gathering statistics on algorithm performance.
  - `src/experiment_utils/`: Utilities relating to experiments and visualization without being used directly by the algorithms. (Only experiments and vizualizations should depend on this.)
- `src_old/`: Old source code, kept for reference, but became unmaintainable.
- `test/`: Unit tests of the code in `src/`.
- `test_robots/`: Contains 3D models and robot descriptions for use in the experiments and vizualizations.
- `analysis/`: Code for analyzing the results of experiments.