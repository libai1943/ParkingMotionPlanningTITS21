# Parking Motion Planning via Lightweight Iterative Optimization

MATLAB implementation of an optimization-based trajectory planner for autonomous parking in environments with irregularly placed obstacles.

This repository contains the source code associated with the paper:

**Optimization-Based Trajectory Planning for Autonomous Parking With Irregularly Placed Obstacles: A Lightweight Iterative Framework**

The planner combines a search-based coarse trajectory generator with a lightweight iterative optimization framework. A **Fault-Tolerant Hybrid A\*** (FTHA) method first provides a guiding trajectory and determines a reasonable homotopy class. The coarse result is then refined by the **Light-weighted Iterative Optimization Method (LIOM)**, where collision avoidance is represented through compact safe travel corridor constraints and the trajectory is iteratively optimized using nonlinear programming.

> **If you use this repository, its implementation, benchmark cases, or the underlying planning method in your research, please cite the associated IEEE Transactions on Intelligent Transportation Systems paper. See [Citation](#citation).**

---

## Overview

Autonomous parking in unstructured environments is difficult because the vehicle must simultaneously satisfy:

- collision-avoidance requirements for irregular obstacles;
- nonholonomic vehicle kinematics;
- steering, velocity, acceleration, and steering-rate limits;
- prescribed initial and terminal vehicle configurations; and
- trajectory quality and computational efficiency requirements.

Directly formulating all obstacle-avoidance relationships inside a large nonlinear optimal control problem can lead to a computationally expensive problem.

This implementation follows a **search-then-optimize** philosophy:

```text
Parking benchmark
      |
      v
Parameter / map initialization
      |
      v
Fault-Tolerant Hybrid A* (FTHA)
      |
      v
Coarse guiding trajectory
      |
      v
Initial guess generation
      |
      v
Safe Travel Corridor construction
      |
      v
Light-weighted Iterative Optimization (LIOM)
      |
      v
AMPL + Ipopt
      |
      v
Optimized parking trajectory
```

The important idea is that the coarse trajectory does not need to be the final high-quality solution. Its primary role is to provide useful topological and geometric guidance for the subsequent optimization.

---

# Main Method

## 1. Fault-Tolerant Hybrid A* for Coarse Planning

The first planning stage is implemented through:

```matlab
SearchTrajectoryViaFTHA();
```

Conventional Hybrid A* is suitable for car-like motion planning because vehicle orientation and nonholonomic motion are considered during state expansion. However, a conventional search may fail to reach the exact terminal configuration within a limited search budget.

The FTHA strategy is designed to provide a usable guiding route even in such cases.

During the search, a best-so-far node is maintained. If the Hybrid A* search successfully reaches the goal, the resulting trajectory can directly serve as the coarse solution. If the search terminates before reaching the goal, the available Hybrid A* portion can be connected toward the target using a lower-dimensional A* search.

The resulting coarse trajectory mainly provides:

1. a collision-aware route from the initial configuration toward the parking target;
2. information about which side of each obstacle the vehicle should pass;
3. a useful initial trajectory for numerical optimization; and
4. the geometric basis for constructing safe travel corridors.

The coarse trajectory is therefore a **guide for optimization**, rather than the final trajectory that the vehicle is expected to track.

---

## 2. Safe Travel Corridors

A central idea of the method is to avoid repeatedly imposing direct vehicle-to-obstacle geometric relationships inside the nonlinear program.

Instead, safe local regions are constructed around the coarse or currently optimized trajectory.

The vehicle is represented using two disks located along its longitudinal direction. For each discretized trajectory point, safe rectangular regions are generated for the front and rear disk centers.

These regions are stored in:

```text
STC_fron
STC_rear
```

The resulting collision-avoidance constraints become simple box constraints such as

```text
xmin <= x_front <= xmax
ymin <= y_front <= ymax

xmin <= x_rear  <= xmax
ymin <= y_rear  <= ymax
```

rather than explicitly checking the full vehicle polygon against every obstacle inside the NLP.

This greatly reduces the complexity of the optimization problem.

---

## 3. Light-weighted Iterative Optimization Method

The main optimization stage is called through:

```matlab
OptimizeTrajectoryViaLIOM();
```

LIOM repeatedly performs the following operations:

```text
current trajectory
      |
      v
construct / update safe travel corridors
      |
      v
form lightweight NLP
      |
      v
solve using Ipopt
      |
      v
obtain improved trajectory
      |
      +--------> next iteration
```

The safe corridors are therefore not permanently fixed from the initial coarse path. They can be reconstructed according to the latest trajectory, allowing the optimizer to progressively exploit more useful free space.

Another important feature can be seen directly in `NLP.mod`: nonlinear vehicle-model consistency equations are incorporated into the objective through quadratic penalty terms, while simple bounds and corridor constraints remain explicit constraints.

The optimization objective includes the terminal time together with penalties related to acceleration, steering angle, steering rate, discretized kinematic consistency, and the consistency between the vehicle pose and the two disk centers.

This formulation keeps each intermediate optimization problem comparatively lightweight.

---

# Getting Started

Clone the repository:

```bash
git clone https://github.com/libai1943/ParkingMotionPlanningTITS21.git
cd ParkingMotionPlanningTITS21
```

Open MATLAB and set the repository directory as the current working directory.

Run:

```matlab
RunMe
```

The original `RunMe.m` executes all **115 benchmark cases**:

```matlab
for ii = 1 : 115
    params_.user.case_id = ii;
    InitializeParams();
    SearchTrajectoryViaFTHA();
    OptimizeTrajectoryViaLIOM();
end
```

The benchmark scenarios are stored in:

```text
ParkingBenchmarks/
```

as:

```text
CaseNo_1.mat
CaseNo_2.mat
...
CaseNo_115.mat
```

---

## Running a Single Case

For debugging or visualization, it is usually more convenient to run only one benchmark.

For example:

```matlab
close all;
clc;
clear global params_;
clear all;

global params_

params_.user.case_id = 1;

InitializeParams();
SearchTrajectoryViaFTHA();
OptimizeTrajectoryViaLIOM();
```

Change

```matlab
params_.user.case_id = 1;
```

to select another benchmark.

---

# Important Functions and Files

A considerable portion of the original research implementation is distributed as MATLAB `.p` files. These are executable MATLAB P-code files whose internal source is protected. Their roles can nevertheless be understood from the overall calling structure, interfaces, accompanying MATLAB code, and optimization model.

## `RunMe.m`

Main entry point of the repository.

```matlab
InitializeParams();
SearchTrajectoryViaFTHA();
OptimizeTrajectoryViaLIOM();
```

It sequentially evaluates all 115 parking benchmark cases.

This is the best place to start when trying to understand the overall execution flow.

---

## `InitializeParams.m`

Initializes the complete planning problem.

Its responsibilities include:

- loading the selected benchmark through `LoadCase()`;
- defining the map boundary;
- defining vehicle dimensions;
- specifying velocity, acceleration, steering, and steering-rate limits;
- configuring Hybrid A* parameters;
- configuring LIOM/NLP parameters;
- constructing occupancy maps;
- inflating obstacles according to the two-disk vehicle approximation; and
- writing parameters required by the AMPL optimization model.

Important vehicle parameters include:

```matlab
params_.vehicle.lw       % wheelbase
params_.vehicle.lf       % front overhang
params_.vehicle.lr       % rear overhang
params_.vehicle.lb       % vehicle width

params_.vehicle.vmax     % maximum velocity
params_.vehicle.amax     % maximum acceleration
params_.vehicle.phymax   % maximum steering angle
params_.vehicle.wmax     % maximum steering rate
```

The current configuration uses:

```matlab
params_.opti.nfe = 200;
params_.opti.max_iter = 5;
```

where `nfe` determines the number of trajectory discretization points used by the optimization model.

### `CreateCostmaps()`

A local function inside `InitializeParams.m`.

It converts polygonal obstacles into an occupancy grid and generates both:

```matlab
params_.scenario.original_map
params_.scenario.dilated_map
```

The second map enlarges occupied regions according to the radius of the disk representation of the vehicle.

MATLAB functions such as `strel` and `imdilate` are used here.

### `ConvertXyToId()`

Converts Cartesian coordinates into occupancy-grid indices while ensuring that the resulting indices remain inside the map boundaries.

---

## `LoadCase.p`

Loads the benchmark specified by:

```matlab
params_.user.case_id
```

The corresponding `.mat` file supplies the parking task and obstacle configuration.

Benchmark data are located in:

```text
ParkingBenchmarks/
```

---

## `SearchTrajectoryViaFTHA.p`

Implements the **Fault-Tolerant Hybrid A\*** stage.

This function generates the coarse trajectory used to initialize the optimization process.

Its purpose is not to produce the final optimal trajectory. Instead, it determines a reasonable collision-free or collision-aware route through the parking environment and provides the homotopy information required by the optimization stage.

---

## `SearchAStarPath.p`

Provides the A* path-search functionality used by the coarse planning process.

Within the FTHA framework, a lower-dimensional A* search can provide supplementary connectivity toward the target when the Hybrid A* component cannot directly obtain the complete terminal configuration within the available search process.

---

## `ResamplePath.p`

Resamples a geometric path.

Search-based paths generally have discretization characteristics determined by the search process, whereas optimization requires a structured set of trajectory points.

This function converts the searched path into a more suitable representation for subsequent trajectory initialization and optimization.

---

## `ResampleCollocationPoints.p`

Resamples the trajectory according to the collocation/discretization requirements of the optimal control problem.

This helps establish the fixed-size trajectory representation used by the AMPL model.

---

## `FormInitialGuessViaFullConfig.p`

Forms the initial guess supplied to the nonlinear optimizer.

A good initial guess is especially important for nonconvex parking trajectory optimization. The coarse search result provides the path topology, while this function organizes the required state and control quantities into the representation used by the NLP.

The resulting initialization is written to:

```text
ig.INIVAL
```

---

## `ConstructSafeTravelCorridors.p`

Constructs the safe travel corridors used to represent collision avoidance.

The implementation generates corridor bounds for the two disk centers representing the ego vehicle.

The resulting data are written into:

```text
STC_fron
STC_rear
```

These files contain the box bounds imposed on the front and rear disk centers at each trajectory discretization point.

This is one of the most important components of the lightweight formulation.

---

## `OptimizeTrajectoryViaLIOM.p`

Main implementation of the **Light-weighted Iterative Optimization Method**.

This function coordinates the iterative trajectory optimization process, including operations such as:

- generating or updating the optimization initialization;
- constructing safe travel corridors;
- calling the AMPL/Ipopt nonlinear optimizer;
- loading the optimized trajectory;
- evaluating the current result; and
- updating the trajectory and corridors for the next LIOM iteration.

This function corresponds to the main optimization stage of the associated paper.

---

## `CreateVehiclePolygon.m`

Generates the rectangular vehicle footprint for a specified vehicle configuration:

```matlab
V = CreateVehiclePolygon(x, y, theta, resolution);
```

Inputs are:

```text
x, y        vehicle reference-point position
theta       vehicle heading
resolution  number of samples used along the footprint boundary
```

The function computes the four corners of the vehicle using the wheelbase, front overhang, rear overhang, and vehicle width.

It is useful for geometric collision checking and visualization.

---

## `DrawParkingScenario.p`

Visualization utility for drawing the parking environment, including the obstacle layout and the associated parking scenario.

---

## `DrawTrajFootprints.p`

Draws vehicle footprints along a planned trajectory.

This is useful for visually inspecting whether the generated trajectory maintains reasonable clearance from obstacles throughout the parking maneuver.

---

## `GenerateGifForLimo.p`

Generates an animation/GIF associated with the LIOM optimization process.

It can be used to visualize how the trajectory evolves during iterative optimization.

---

## `RegulateAngle.m`

Normalizes an angle into:

```text
[0, 2*pi]
```

This small utility is used to maintain a consistent heading representation.

---

## `WriteBasicParameterFile.m`

Exports the MATLAB-side planning parameters into:

```text
BasicParameters
```

for use by the AMPL model.

The exported parameters include:

- map boundaries;
- vehicle dimensions;
- dual-disk geometry;
- velocity limit;
- acceleration limit;
- steering-angle limit;
- steering-rate limit;
- number of finite elements; and
- objective-function weights.

This file forms part of the MATLAB-to-AMPL interface.

---

## `WriteBoundaryValues.m`

Writes the initial and terminal parking configurations into:

```text
SixBoundaryValues
```

The six values are:

```text
x0
y0
theta0

xf
yf
thetaf
```

These values are subsequently loaded by `NLP.mod`.

---

## `NLP.mod`

The central AMPL nonlinear programming model.

Its optimization variables include:

```text
tf          terminal time

x, y        vehicle position
theta       vehicle heading
v           longitudinal velocity
a           longitudinal acceleration
phy         steering angle
w           steering rate

xf, yf      front disk center
xr, yr      rear disk center
```

The objective contains the terminal time and regularization terms for:

```text
acceleration
steering angle
steering rate
kinematic consistency
front/rear disk pose consistency
```

The model enforces:

- initial state conditions;
- terminal state conditions;
- velocity bounds;
- acceleration bounds;
- steering-angle bounds;
- steering-rate bounds; and
- front/rear safe-corridor box constraints.

The collision-avoidance conditions therefore appear in a particularly simple form:

```ampl
STC_front[i,1] <= xf[i] <= STC_front[i,2];
STC_front[i,3] <= yf[i] <= STC_front[i,4];

STC_rear[i,1] <= xr[i] <= STC_rear[i,2];
STC_rear[i,3] <= yr[i] <= STC_rear[i,4];
```

This compact constraint structure is a major reason the intermediate optimal-control problem remains lightweight.

---

## `rr.run`

AMPL execution script.

The script:

1. loads `NLP.mod`;
2. loads the initial solution from `ig.INIVAL`;
3. selects Ipopt as the nonlinear programming solver;
4. solves the NLP;
5. writes the optimized solution back as the next initial guess; and
6. exports optimization results to `AmplResults/`.

For example:

```ampl
option solver ipopt;
solve;
```

After optimization, trajectory data are stored in files such as:

```text
AmplResults/x.txt
AmplResults/y.txt
AmplResults/theta.txt
AmplResults/v.txt
AmplResults/a.txt
AmplResults/phy.txt
AmplResults/w.txt
AmplResults/terminal_time.txt
AmplResults/infeasibility.txt
```

---

## `ipopt.opt`

Ipopt configuration file.

The current repository specifies settings including:

```text
max_iter        500
max_cpu_time    1
tol             1e-7
mu_strategy     adaptive
linear_solver   ma27
```

These settings were selected for the original research implementation.

---

## `BasicParameters`

Intermediate parameter file generated from MATLAB and loaded by AMPL.

Normally, users should modify the corresponding definitions in:

```matlab
InitializeParams.m
```

rather than manually editing this file.

---

## `SixBoundaryValues`

Intermediate file containing the initial and terminal parking configurations.

It is generated by:

```matlab
WriteBoundaryValues();
```

---

## `STC_fron` and `STC_rear`

Safe travel corridor data for the front and rear disk centers.

These files are consumed directly by the AMPL model and are regenerated during the planning process.

They should generally be treated as intermediate files rather than manually edited inputs.

---

## `ig.INIVAL`

Initial-value file supplied to AMPL/Ipopt.

The optimized solution can also be written back into this file, enabling warm starting during the iterative optimization procedure.

---

## `AmplResults/`

Stores numerical results exported from AMPL.

The directory contains the optimized states, controls, terminal time, and an infeasibility measure.

---

## `SaveFigure.m`

Simple utility for saving the current MATLAB figure:

```matlab
SaveFigure('result.png');
```

---

## `Arrow.p`

Internal graphical utility used by the original implementation.

It is not a primary planning interface and normally does not need to be called directly.

---

## `asd.p`

Internal auxiliary P-code used by the research implementation.

Users generally do not need to interact with this function directly.

---

# Repository Structure

A simplified view of the repository is:

```text
ParkingMotionPlanningTITS21/
│
├── RunMe.m
├── InitializeParams.m
├── LoadCase.p
│
├── SearchTrajectoryViaFTHA.p
├── SearchAStarPath.p
├── ResamplePath.p
│
├── FormInitialGuessViaFullConfig.p
├── ResampleCollocationPoints.p
├── ConstructSafeTravelCorridors.p
├── OptimizeTrajectoryViaLIOM.p
│
├── CreateVehiclePolygon.m
├── RegulateAngle.m
│
├── DrawParkingScenario.p
├── DrawTrajFootprints.p
├── GenerateGifForLimo.p
├── SaveFigure.m
├── Arrow.p
│
├── WriteBasicParameterFile.m
├── WriteBoundaryValues.m
│
├── NLP.mod
├── rr.run
├── ipopt.opt
│
├── BasicParameters
├── SixBoundaryValues
├── STC_fron
├── STC_rear
├── ig.INIVAL
│
├── AmplResults/
│
├── ParkingBenchmarks/
│   ├── CaseNo_1.mat
│   ├── ...
│   └── CaseNo_115.mat
│
├── ampl.exe
├── ipopt.exe
├── libhsl.dll
├── libipoptfort.dll
└── LICENSE
```

---

# Software Requirements

The implementation is MATLAB-based and the original repository also contains the AMPL/Ipopt components used to solve the nonlinear programs.

The current code is particularly oriented toward a **Windows** environment because the repository contains Windows executables and DLLs and `rr.run` uses Windows-style shell commands such as:

```text
del
```

Main software components are:

- MATLAB;
- MATLAB Image Processing Toolbox, used by functions such as `strel` and `imdilate`;
- AMPL;
- Ipopt;
- HSL/MA27 linear solver support used by the supplied Ipopt configuration.

The repository already contains several binaries and libraries used by the original implementation.

---

# Modifying the Planner

Most user-level parameters can be found in:

```matlab
InitializeParams.m
```

For example, vehicle parameters are defined through:

```matlab
params_.vehicle.lw
params_.vehicle.lf
params_.vehicle.lr
params_.vehicle.lb

params_.vehicle.vmax
params_.vehicle.amax
params_.vehicle.phymax
params_.vehicle.wmax
```

Hybrid A* parameters are under:

```matlab
params_.hybrid_astar
```

and optimization parameters are under:

```matlab
params_.opti
```

Examples include:

```matlab
params_.opti.nfe
params_.opti.acceptance_tolerance
params_.opti.cost_function_external_penalty_weight
params_.opti.cost_a
params_.opti.cost_w
params_.opti.cost_phy
params_.opti.max_iter
```

When experimenting with these parameters, it is recommended to begin with a single benchmark case rather than running all 115 cases.

---

# Notes on the P-Code Files

Several core modules are released as MATLAB `.p` files.

MATLAB P-code is executable but does not expose the original `.m` source implementation. This repository therefore provides a mixture of:

```text
editable MATLAB source code
+
protected MATLAB P-code
+
AMPL optimization model
+
benchmark data
+
solver interface files
```

The visible `RunMe.m`, parameter initialization code, AMPL formulation, input/output files, and function interfaces make the overall algorithmic pipeline explicit, while several implementation details of the original planner remain encapsulated in the P-code modules.

---

# Citation

**Please cite the paper if you use this repository, the benchmark cases, or the underlying FTHA/LIOM planning framework in academic work.**

### IEEE Style

B. Li, T. Acarman, Y. Zhang, et al., “Optimization-based trajectory planning for autonomous parking with irregularly placed obstacles: A lightweight iterative framework,” *IEEE Transactions on Intelligent Transportation Systems*, vol. 23, no. 8, pp. 11970–11981, Aug. 2022.

DOI: `10.1109/TITS.2021.3109011`

### BibTeX

```bibtex
@article{li2022optimization,
  title={Optimization-Based Trajectory Planning for Autonomous Parking With Irregularly Placed Obstacles: A Lightweight Iterative Framework},
  author={Li, Bai and Acarman, Tankut and Zhang, Youmin and Ouyang, Yakun and Yaman, Cagdas and Kong, Qi and Zhong, Xiang and Peng, Xiaoyan},
  journal={IEEE Transactions on Intelligent Transportation Systems},
  volume={23},
  number={8},
  pages={11970--11981},
  year={2022},
  publisher={IEEE},
  doi={10.1109/TITS.2021.3109011}
}
```

If you build upon the code or use the 115 parking benchmark cases in a publication, citing the paper is appreciated and helps acknowledge the research effort behind the repository.

---

# License

This repository is released under the **GNU General Public License v3.0 (GPL-3.0)**.

See the `LICENSE` file for details.
