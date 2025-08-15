# Whole-Body Trajectory Optimization in the SE(3) Tangent Space

## Overview

Agile whole-body motion generation for legged and humanoid robots remains a fundamental challenge in robotics. This repository implements a trajectory optimization framework that formulates the robot's floating-base dynamics in the tangent space of SE(3), enabling efficient optimization using standard off the self NLP solvers (IPOPT) without requiring manifold-specific techniques.

The implementation supports whole-body dynamics, contact constraints and terrain modeling while leveraging analytical derivatives via the Pinocchio library. 

See more at [https://lar.upatras.gr/projects/ibrics.html](https://lar.upatras.gr/projects/ibrics.html).

## Maintainers

- Evangelos Tsiatsianas (University of Patras) - etsiatsianas@ac.upatras.gr
- Konstantinos Chatzilygeroudis (University of Patras) - costashatz@upatras.gr

## Citing our work

If you use this code in a scientific publication, please use the following citation:

```bibtex
@inproceedings{tsiatsianas2025comparative,
      title={{A Comparative Study of Floating-Base Space Parameterizations for Agile Whole-Body Motion Planning}},
      author={Tsiatsianas, Evangelos and Kiourt, Chairi and Chatzilygeroudis, Konstantinos},
      booktitle={IEEE-RAS International Conference on Humanoid Robots (Humanoids)},
      year={2025}
    }
```

## Install on your system

### Installation
We recommend using [Conda](https://docs.conda.io/) for managing dependencies.

#### Create and activate the environment
- `conda create -n se3_trajopt python=3.13`
- `conda activate se3_trajopt`

#### Install dependencies
Use `conda install -c "package_name"` to install the following packages:
  - conda-forge::pinocchio
  - conda-forge::meshcat-python
  - conda-forge::cyipopt
  - conda-forge::example-robot-data
  - conda-forge::matplotlib

### Usage

- `conda activate se3_trajopt`
- `export PYTHONPATH=$(pwd)/src`
- `python src/examples/talos_trajopt.py --vis`

use `--vis` option to visualize the results with meshcat
