# GSTLBO Algorithm for Heterogeneous UAV Swarm Cooperative Task Allocation

[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/downloads/)
![Code](https://img.shields.io/badge/full%20code-released%20upon%20acceptance-orange.svg)

> This repository is the official project page of the paper: **"Structure-Guided Construction and Search for Heterogeneous UAV Swarm Cooperative Task Allocation with Temporal Task Chains"**.
>
> **Note**: Currently, the benchmark instances, the HIL ground-station data and the data-generation scripts are provided. **The full source code — the GSTLBO solver, all baseline implementations, and the experiment, evaluation and visualization scripts — will be open-sourced upon paper acceptance.**

---

## 📋 Table of Contents
- [About the Project](#about-the-project)
- [Key Features](#key-features)
- [Repository Contents](#repository-contents)
- [Environment Requirements](#environment-requirements)
- [Usage](#usage)
- [Results](#results)
- [Citation](#citation)
- [Acknowledgments](#acknowledgments)

---

## 🚀 About the Project

This work addresses the **heterogeneous UAV swarm cooperative task allocation (HUSCTA) problem with temporal task chains**, where capability heterogeneity and target-level precedence constraints are tightly coupled. We propose a **Grouping and Structure-Guided Teaching-Learning-Based Optimization (GSTLBO)** framework to efficiently solve this NP-hard problem. The methodological details are described in the paper.

The repository currently includes:

- Benchmark instances for six problem scales (20 instances per scale)
- HIL ground-station scenario data
- Data-generation scripts
- (Full code coming soon after paper acceptance)

---

## ✨ Key Features

- 🧩 **Structure-Supplying Construction (GSNN)**: Generates a naturally feasible solution that explicitly specifies the formation grouping, the spatial target partitioning and the shared visitation order.
- 🎯 **Structure-Guided Search (IntraVND + InterANS)**: Variable-neighborhood search over shared visitation orders within teams, and adaptive neighborhood search that reallocates complete target chains across teams.
- 🔗 **Complex Constraints Handling**: Natively supports temporal task chains, stage-wise capability matching, flight-endurance limits and no-fly zones.
- 🛡️ **No-Fly-Zone Avoidance**: Tangent detours under the Euclidean metric, and kinematically feasible Dubins rerouting that respects the minimum turning radius under the Dubins metric.
- ⚡ **Online Replanning**: Team-constrained insertion (TCI) for dynamically appearing targets.
- 📊 **Comprehensive Benchmarks and Baselines**: Six scales from 10 to 200 targets; comparisons against CBBA, DCTA, MTLBO, AMTLBO, TBGA, AMA, RSALNS, ADGVNS, and an OR-Tools CP-SAT reference on the three smallest scales.
- ✈️ **Hardware-in-the-Loop Validation**: Closed-loop experiments on real flight hardware with a ground control station, covering multi-base, dense no-fly-zone and dynamic-target scenarios.

---

## 📁 Repository Contents

| Path | Contents | Status |
|------|----------|--------|
| `data/` | Benchmark instances (21 `.npz` files; six scales, four experiment families) | ✅ Available |
| `data/hil/` | HIL ground-station scenario files (30 `.plan`) | ✅ Available |
| `scripts/` | Data-generation scripts (benchmark families and ground-station scenarios) | ✅ Available |
| `code/` | GSTLBO solver, baseline implementations, evaluation and visualization | 🚧 Upon acceptance |

For the full field-level description of the data and the usage of the generation scripts, see **[`data/README.md`](data/README.md)**.

---

## 🛠️ Environment Requirements

- **Python 3.8 or higher**
- Dependencies:

  ```
  numpy
  scipy
  dubins
  matplotlib
  ortools      # only for the CP-SAT reference solver (small scales)
  ```

  **Note for Windows users**: If you encounter issues installing `dubins`, please refer to [this guide](https://blog.csdn.net/qq_28266955/article/details/80332909) for detailed installation instructions.

---

## 🎮 Usage

### 1. Benchmark Data (available now)

```python
import numpy as np

# Load the M16-N100 main benchmark (20 instances)
d = np.load("data/n100_m16_randdepot_real.npz")
print(d.files)

locs   = d["locs"]           # (20, 100, 2)  target positions, ENU metres
depot  = d["depot"]          # (20, 2)       base position per instance
types  = d["drone_types"]    # (20, 16)      0/1/2 = reconnaissance/strike/evaluation
```

The data-generation scripts and the HIL scenario files are described in [`data/README.md`](data/README.md).

### 2. (Coming Soon) Run GSTLBO

```bash
python gstlbo_v2.py --use_npz_data --npz_file_path "./data/n100_m16_randdepot_real.npz" \
    --npz_data_limit 20 --population_size 50 --num_classes 2 --max_iterations 500 \
    --initialization_strategy gsnn --distance_type euclidean --objective_type weighted_sum \
    --enable_joint_aligned_2opt --enable_adaptive_weights --acceleration 3.0 --seed 42 \
    --accept_strategy sa --sa_temperature 0.05 --sa_cooling_rate 0.98 \
    --sgvnd_organization vnd --no_sgvnd_joint_2opt_only
```

---

## 📊 Results

### Performance Overview

- GSTLBO achieves the **lowest average total distance and makespan** among the compared heuristic methods on all six problem scales.
- Under equal per-instance wall-clock budgets (5 / 15 / 30 / 60 s), GSTLBO attains the **lowest average normalized objective** at every scale and every budget.
- Controlled decomposition shows that the group-level structure supplied by the GSNN construction accounts for the main gains, while the structure-guided search operators further improve the best constructed solution by **5.6%–11.5%**.
- Extension experiments validate no-fly-zone avoidance under the Dubins metric and online replanning for dynamically appearing targets; hardware-in-the-loop experiments on six real flight kits verify end-to-end feasibility.

---

## 📝 Citation

If you find this code useful in your research, please cite our paper:

```bibtex
@article{wei2026gstlbo,
  title   = {Structure-Guided Construction and Search for Heterogeneous UAV Swarm Cooperative Task Allocation with Temporal Task Chains},
  author  = {xxx},
  journal = {XX},
  year    = {2026},
  volume  = {XX},
  pages   = {XX--XX}
}
```

---

## 🙏 Acknowledgments

We sincerely thank the following open-source projects for their valuable code and inspiration:

- [TLC-CBBA](https://github.com/ycchao0406/TLC_CBBA) - For the CBBA baseline implementation reference
- [CPMCTA-AMTLBO](https://github.com/yuxinyongMath16/CPMCTA-AMTLBO) - For the AMTLBO baseline implementation reference
- [PARCO](https://github.com/ai4co/parco) - For the instance (NPZ) format and test-case generation framework reference

We also thank the authors of the compared baseline algorithms for making their methods available to the community.

---

⭐ If this project helps you, please give it a star!
