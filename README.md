# P45 — Active Vibration Control of a Circular Piezoelectric Plate

P45 is a research codebase for **finite element method (FEM) simulation and active vibration control** of a circular piezoelectric plate. The project models a plate with distributed piezoelectric actuators and capacitive sensors, and implements feedback and feedforward control strategies to achieve precise displacement targets.

---

## Table of Contents

1. [Physical System](#physical-system)
2. [Project Structure](#project-structure)
3. [Dependencies & Installation](#dependencies--installation)
4. [Running Simulations](#running-simulations)
5. [Module Reference](#module-reference)
6. [Simulation Workflow](#simulation-workflow)
7. [Output Artifacts](#output-artifacts)
8. [Key Parameters](#key-parameters)

---

## Physical System

The simulated structure is an **annular circular plate** with the following properties:

| Property | Value |
|---|---|
| Outer radius | 120 mm |
| Inner radius (clamped hub) | ~28 mm |
| Thickness | 1.61 mm |
| Material | Aluminum-like (E = 91 GPa, ν = 0.24, ρ = 2530 kg/m³) |
| Plate theory | Mindlin (includes shear deformation) |
| Number of actuators | 45 (distributed on a polar grid) |
| Sensor type | Capacitive displacement sensors co-located with actuators |
| Time step | 12.5 µs (80 kHz sampling) |

The plate is clamped at the inner radius and free at the outer edge. The 45 point actuators apply transverse forces via piezoelectric elements. The capacitive sensors measure local displacement; a quadratic model converts displacement and rotation to capacitance:

```
C = cap_z * w + cap_a1 * (θx² + θy²) + C₀
```

where `cap_z = 310 nF/m`, `cap_a1 = 9.01×10⁻¹¹ F`, `C₀ = 31.89 pF`.

---

## Project Structure

```
P45/
├── control/                          # Main closed-loop control simulation
│   ├── main.py                       # Entry point: FEM assembly + time integration + control
│   ├── actuators.py                  # ActList class: sensor/actuator abstraction
│   ├── feedback.py                   # FeedBack class: PD feedback controller
│   ├── feedforward.py                # FeedForward class: predictive trajectory controller
│   ├── integrators.py                # Eulero, Mantegazza, SensorFilter classes
│   ├── condense.py                   # Model order reduction (exports Kred.npy, Mred.npy)
│   ├── geom/                         # Gmsh geometry file (geom.geo)
│   ├── data/                         # Actuator coordinates and reduced matrices (*.npy)
│   └── results/                      # MATLAB plotting scripts
│
├── modal/                            # Free-vibration (eigenvalue) analysis
│   └── plate.py                      # Computes natural frequencies and mode shapes
│
├── direct_implicit/                  # Comparison of direct vs. implicit time-stepping
│   ├── plate.py                      # Direct (explicit Euler) solver
│   └── platem.py                     # Implicit (Mantegazza) solver
│
├── space/                            # Spatial discretization study
│   └── main.py                       # Frequency sweep / spatial convergence checks
│
├── time_history/                     # Simplified time-domain baseline
│   └── main.py                       # Basic feedback/feedforward, single control gain
│
├── time_history_2_order_filter/      # Time-domain with 2nd-order sensor filter
│   └── main.py
│
├── time_history_3/                   # Extended time-domain analysis
│   └── main.py
│
├── time_history_for_data_extraction/ # Time-domain optimised for extracting model data
│   └── main.py
│
├── time_history_with_act_dyn/        # Time-domain including actuator dynamics
│   └── main.py
│
├── time_history_with_act_dyn_expl/   # Explicit time-stepping with actuator dynamics
│   └── main.py
│
├── for_data_extraction/              # Parameter identification / least-squares fitting
│   └── main.py
│
├── voltage/                          # Voltage-based piezoelectric sensing
│   ├── main.py                       # Voltage observer + control loop
│   └── VoltageObserver.m             # MATLAB observer design
│
├── pippo/                            # Minimal reference / sanity-check implementation
│   └── main.py
│
└── LICENSE
```

---

## Dependencies & Installation

### System packages

```bash
# Ubuntu / Debian (tested with Ubuntu 16.04 LTS — see legacy note below)
sudo apt-get install fenics gmsh python-dolfin python-petsc4py
```

> **Legacy note:** The code targets the **FEniCS 2017.x / DOLFIN** API and is written in **Python 2**, which reached end-of-life in January 2020. Ubuntu 16.04 LTS (the original development environment) also reached end-of-life in April 2021. Running this codebase on a modern system requires either a legacy Docker image (e.g. `quay.io/fenicsproject/stable:2017.2.0`) or manual porting to the FEniCS 2019.x Python 3 API. The Python 2-specific idioms (`print` as a statement, `MixedFunctionSpace`, integer division) will need to be updated for Python 3 compatibility.

### Python packages

```bash
pip install numpy matplotlib scipy ipython
```

### Optional (post-processing)

- **MATLAB / GNU Octave** — for the `.m` plotting scripts in `results/` and `data/`
- **ParaView** — for 3D visualisation of `.pvd` solution snapshots

---

## Running Simulations

Each subdirectory is self-contained and must be executed from **within that directory**:

```bash
cd control/
python main.py
```

The script will:
1. Write the mesh resolution parameters into `geom/geom.geo`.
2. Call `gmsh` to generate `geom/geom.msh`.
3. Call `dolfin-convert` to produce `geom/mesh.xml`.
4. Assemble the FEM matrices (K, M, C).
5. Run the explicit Euler initialisation step, then advance with the Mantegazza integrator.
6. Apply feedforward and feedback forces at each time step.
7. Write ParaView snapshots to `paraview/` and displacement data to `*.dat` files.

### Before running `control/main.py`

The feedforward controller requires pre-computed reduced model matrices. Run the condensation script first:

```bash
cd control/
python condense.py      # generates data/Kred.npy and data/Mred.npy
```

---

## Module Reference

### `actuators.py` — `ActList` class

Manages the 45 co-located actuator/sensor points.

| Method | Description |
|---|---|
| `__init__(V, coords)` | Binds actuator coordinates to FEM mesh nodes |
| `disp2cap_all(sol)` | Converts FEM solution to capacitance vector (nF) |
| `cap2disp_all(cap)` | Converts capacitance vector to displacement (m), with ADC noise |
| `cap2disp_all_clean(cap)` | Same conversion without noise (ideal sensors) |
| `disp2disp_all(sol)` | Full round-trip: displacement → capacitance → noisy displacement |
| `dp_all()` | Returns a `Measure` pointing to all actuator vertices (used for force assembly) |
| `point(i)` | Returns a `Point` object for actuator `i` |
| `plot_coords()` | 3D scatter plot of actuator positions |

Standalone sensor functions:

| Function | Description |
|---|---|
| `disp2cap(w, tx, ty)` | Scalar displacement + rotations → capacitance |
| `cap2disp(cap)` | Scalar capacitance → displacement (ideal) |
| `digital_cap2disp(cap)` | Scalar capacitance → displacement (15-bit ADC + noise) |

---

### `feedback.py` — `FeedBack` class

Proportional-Derivative (PD) controller acting on the 45 actuator positions.

```python
FeedBack(actlist, dt, gp=10000*ones(45), gd=35*ones(45), capacitor_emulate=False)
```

| Parameter | Default | Description |
|---|---|---|
| `actlist` | — | `ActList` instance |
| `dt` | — | Simulation time step (s) |
| `gp` | `10000` (×45) | Proportional gain vector |
| `gd` | `35` (×45) | Derivative gain vector |
| `capacitor_emulate` | `False` | If `True`, reads sensors through the capacitive model (with noise) |

**`force(uold, udold, uref, udref)`**  
Returns the 45-element force vector:  `F = gp*(uref - u) + gd*(udref - ud)`

---

### `feedforward.py` — `FeedForward` class

Pre-computes forces to drive the plate to a desired static displacement profile using a smooth S-curve ramp trajectory.

```python
FeedForward(u, T, capacitanceread=False)
```

| Parameter | Description |
|---|---|
| `u` | Desired displacement vector at actuator locations (m) |
| `T` | Total trajectory period (s); ramp-up occupies the first T/4 |
| `capacitanceread` | If `True`, loads `data/Kred_with_cond.npy` (sensor-distorted stiffness) |

**Trajectory shape** — cosine ramp over `[0, T/4]`, constant at `u` for `[T/4, T]`:

```
w(t)  = u * 0.5*(1 - cos(4π/T * t))    for t < T/4
w(t)  = u                               for t ≥ T/4
```

**`force(t)`** — Returns `M·ẅ + C·ẇ + K·w` evaluated at time `t` using the reduced-order model.

---

### `integrators.py` — Time-stepping schemes

#### `Eulero` (explicit, first-order)

Used for the first time step only (bootstrapping).  
Solves: `(M/dt² + C/dt + K) u_n = F + (M/dt² + C/dt) u_{n-1} + (M/dt) u̇_{n-1}`

#### `Mantegazza` (implicit, second-order)

Energy-stable Newmark-family scheme with controllable algorithmic damping.

```python
Mantegazza(V, M, C, K, dt, rho=0.58, alpha=1.0)
```

| Parameter | Description |
|---|---|
| `rho` | Spectral radius at infinite frequency (0 = maximum damping, 1 = no damping) |
| `alpha` | Auxiliary parameter affecting the accuracy / damping trade-off |

Maintains two previous time-steps internally; call `integrate(F)` then `update()` at every step.

#### `SensorFilter` (first-order low-pass)

Discrete-time first-order filter modelling capacitive sensor bandwidth.

```python
SensorFilter(ft, n, dt, x0)
```

| Parameter | Description |
|---|---|
| `ft` | Cut-off frequency (Hz), default 40000 Hz |
| `n` | Number of sensors |
| `dt` | Time step (s) |
| `x0` | Initial state value |

State equation: `ẋ = -ωx + ωu`, where `ω = 2πft`.

---

### `condense.py` — Model order reduction

Builds and saves the 45×45 reduced-order stiffness and mass matrices used by the feedforward controller:

| Output file | Description |
|---|---|
| `data/Kred.npy` | Reduced stiffness matrix (condensed to actuator DOFs) |
| `data/Mred.npy` | Reduced mass matrix (lumped total mass distributed evenly) |
| `data/Kred_with_cond.npy` | Reduced stiffness with capacitance-reading correction |

The condensation follows a **static condensation** approach: the full FEM stiffness is used to compute unit-displacement solutions at each actuator point, and the resulting reaction forces at all actuator nodes form the rows/columns of `Kred`.

---

## Simulation Workflow

```
geom/geom.geo
      │
      ▼ gmsh -2
geom/geom.msh
      │
      ▼ dolfin-convert
geom/mesh.xml
      │
      ▼ FEniCS / DOLFIN
   K, M, C matrices (FEM)
      │
      ├──► condense.py ──► data/Kred.npy, data/Mred.npy
      │
      ▼
   main.py (time loop)
      │
      ├── Euler step (t=0 → t=dt)
      │       │
      │       ▼
      └── Mantegazza steps (t=dt → t=T)
              │
              ├── FeedForward.force(t) — open-loop pre-computed forces
              ├── FeedBack.force(...)  — closed-loop PD correction
              │
              ▼
         u(x,t), u̇(x,t)
              │
              ├──► paraview/w*.pvd   (3D displacement snapshots)
              └──► *.dat             (actuator displacement time series)
```

---

## Output Artifacts

After running `control/main.py`:

| File | Contents |
|---|---|
| `paraview/w0.pvd` … `w50.pvd` | ParaView-compatible transverse deflection snapshots (50 frames) |
| `15u1.dat` | Time (ms) vs. displacement (nm) for actuator 1 |
| `15u2.dat` | Time (ms) vs. displacement (nm) for actuator 45 |
| `15ref1.dat` | Reference trajectory for actuator 1 |
| `15ref2.dat` | Reference trajectory for actuator 45 |
| `15ref1±10.dat` | Reference ± 10 nm tolerance bands |
| `15ref2±10.dat` | Reference ± 10 nm tolerance bands |

The `.dat` files are two-column ASCII (space-separated) and can be plotted directly with MATLAB/Octave scripts in `results/` or with `matplotlib`.

---

## Key Parameters

The most commonly adjusted parameters are set near the top of each `main.py`:

| Parameter | Location | Description |
|---|---|---|
| `rint` | `main.py` | Inner (clamped) radius of the plate (m) |
| `rest` | `main.py` | Outer radius of the plate (m) |
| `res` | `main.py` | Gmsh mesh resolution (m); smaller = finer mesh |
| `dt` | `main.py` | Time step for the main (Mantegazza) integration loop (s) |
| `idt` | `main.py` | Time step for the Euler initialisation sub-loop (s) |
| `T` | `main.py` | Total simulation duration (s) |
| `gp` | `feedback.py` or `main.py` | Proportional gain vector (length 45) |
| `gd` | `feedback.py` or `main.py` | Derivative gain vector (length 45) |
| `c_K`, `c_M` | `main.py` | Rayleigh damping coefficients |
| `rho` | `Mantegazza(...)` | Spectral radius for algorithmic damping |

> **Tip:** Set `res = 3e-3` for the default mesh (~3 mm element size). Increasing `res` speeds up assembly at the cost of accuracy; decreasing it gives a finer mesh but longer assembly time.
