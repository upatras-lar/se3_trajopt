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

## Acknowledgments

This work has been partially supported by project MIS 5154714 of the National Recovery and Resilience Plan Greece 2.0 funded by the European Union under the NextGenerationEU Program.

<p align="center">
<img src="https://archimedesai.gr/images/logo_en.svg" alt="logo_archimedes"/>
<p/>

This work was conducted within the [Laboratory of Automation and Robotics](https://lar.ece.upatras.gr/) (LAR), Department of Electrical and Computer Engineering, and [Archimedes](https://archimedesai.gr/en/), RC Athena, Greece.

<p align="center">
<img src="http://lar.ece.upatras.gr/wp-content/uploads/sites/147/2022/10/lar_profile_alpha.png" alt="logo_lar" width="20%"/><br/>
<img src="https://www.upatras.gr/wp-content/uploads/up_2017_logo_en.png" alt="logo_upatras" width="50%"/>
</p>

