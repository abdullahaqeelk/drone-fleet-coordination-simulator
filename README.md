# Autonomous Drone Fleet Coordination Simulator

A Java simulation of multiple quadrotor drones operating as autonomous agents in a shared 3D environment, coordinating through consensus-based formation control while avoiding collisions and exchanging state over a range-limited, lossy communication channel.

Built as a Complex Engineering Problem (CEP) for an Object-Oriented Programming course.

---

## Problem

Fleets of drones are increasingly deployed for precision crop monitoring, mapping, and spraying. Coordinating them in the field is expensive and risky — a control bug means damaged hardware. Simulation lets coordination algorithms be tested before anything flies.

This project models the collective behaviour of a drone fleet where each drone is an independent autonomous agent: it computes its own control inputs, decides its own motion from local neighbour information, and shares state only with drones inside its communication radius. There is no central commander issuing per-drone instructions.

## Design

The system is modular, with each class corresponding to one physical or computational concern. This separation is the point of the exercise — control logic, physical dynamics, coordination rules, and data recording are independently replaceable.

| Class | Responsibility |
|---|---|
| `Simulator` | Numerical engine — manages time, integrates all drone states, synchronises communication and logging |
| `Drone` | One quadrotor: physical parameters and dynamic response to applied forces |
| `Controller` | PD control law converting position/velocity error into thrust and torque |
| `FormationManager` | Consensus-based coordination from neighbour positions and velocities |
| `CollisionAvoidance` | Repulsive potential field preventing inter-drone overlap |
| `CommunicationModule` | Range-limited, probabilistically lossy state exchange |
| `Environment` | Spatial boundaries and boundary-condition enforcement |
| `Logger` | State recording and performance metric computation |

## Simulation Loop

The simulation advances in discrete time steps of Δt. Each step:

1. `Controller` computes thrust and desired attitude for target trajectories
2. `Drone` applies control inputs to its dynamic model, deriving acceleration and angular velocity
3. `FormationManager` adjusts behaviour from neighbour states to hold formation
4. `CollisionAvoidance` applies repulsive forces where drones approach each other
5. `CommunicationModule` exchanges state among drones in range, subject to packet loss
6. `Simulator` integrates translational and rotational motion for all drones
7. `Logger` records positions, velocities, thrusts, and communication events

All drones are updated simultaneously within a step, keeping kinematic and dynamic state consistent.

## Mathematical Model

### Translational dynamics

Newton's second law, with forces from gravity, thrust, drag, collision avoidance, and formation control:

```
m·a = m·g + R·T + F_aero + F_rep + F_form
```

where `R` is the body-to-world rotation matrix, `T = [0, 0, T_z]ᵀ` the net thrust in body frame, and `g = [0, 0, −9.81]ᵀ`. Aerodynamic drag is linear in velocity:

```
F_aero = −k_d · v
```

Integration is explicit Euler:

```
v(t+Δt) = v(t) + a(t)·Δt
p(t+Δt) = p(t) + v(t+Δt)·Δt
```

### Rotational dynamics

Euler's rotational equation, with the gyroscopic cross-product term:

```
ω̇ = I⁻¹ (τ − ω × (I·ω))
R(t+Δt) = R(t) · Exp(ω·Δt)
```

`I` is the inertia tensor. The rotation matrix is updated on the manifold via the exponential map rather than by adding angles, which avoids drift and gimbal issues.

### Control

Position and velocity error drive a PD law with gravity compensation:

```
e_p = p_desired − p_actual
e_v = v_desired − v_actual

a_cmd = k_p^ctrl · e_p + k_d^ctrl · e_v + g
```

Commanded acceleration becomes an inertial thrust vector, rotated into the body frame, from which the vertical component is extracted:

```
T_inertial = m · a_cmd
T_body     = Rᵀ · T_inertial
T_z        = T_body,z
```

Attitude is regulated by torque:

```
τ = k_R (R_target − R) + k_ω (ω_target − ω)
```

### Formation control

Each drone adjusts toward its neighbours' positions and velocities — a second-order consensus law over the set `N_i` of drones within communication range:

```
F_form,i = − Σ_{j ∈ N_i} [ k_p^form (p_i − p_j) + k_v^form (v_i − v_j) ]
```

The position term aligns relative geometry; the velocity term equalises speeds so the group moves cohesively rather than oscillating.

**Note on notation:** the controller gains `k_p^ctrl`, `k_d^ctrl` and the formation weights `k_p^form`, `k_v^form` are distinct quantities despite similar naming. They are separately named in the source.

### Collision avoidance

Repulsion grows without bound as drones converge:

```
F_rep,i = Σ_{j ∈ N_i} k_rep · (p_i − p_j) / ‖p_i − p_j‖²
```

### Communication

A pair of drones can exchange state only if within range, and each attempt may fail:

```
connected:  ‖p_i − p_j‖ < R_comm
delivered:  rand() > p_loss
```

Because formation and avoidance both depend on neighbour state, packet loss degrades coordination — which is the realistic behaviour being modelled, not a defect.

### Boundaries

Drones are constrained to the operational area:

```
p_x, p_y ∈ [0, width] × [0, height]
```

On reaching a boundary, velocity is reflected or the drone repositioned in bounds.

## Performance Metrics

```
Average spacing        =  1/(N(N−1)) · Σ_{i,j} ‖p_i − p_j‖
Collision count        =  | { (i,j) : ‖p_i − p_j‖ < d_min } |
Comm. success rate     =  successful messages / total messages
```

Together these measure coordination quality, safety, and communication robustness.

## Building and Running

Requires a JDK (17 or later) and JavaFX for the graphical view.

```
1. Import the project into your IDE, or compile from the command line
2. Ensure the JavaFX SDK is on the module path if running the GUI
3. Run the Simulator entry point
```

Simulation parameters — drone count, Δt, environment dimensions, drone
physical constants, control gains — are read from `config.txt` at startup.
Editing that file changes the scenario without recompiling, which keeps runs
reproducible and parameters traceable.

## Output

| File | Contents |
|---|---|
| `positions.csv` | Per-timestep position, velocity, and thrust for every drone |
| `metrics.txt` | Final performance summary |

A sample `positions.csv` is included. Larger mission logs are regenerated by
running the simulator and are not committed.

## Model Sources

The model combines established formulations rather than implementing a single paper:

- **Quadrotor dynamics** — Mahony, R., Kumar, V. & Corke, P. (2012). "Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor." *IEEE Robotics & Automation Magazine* 19(3): 20–32.
- **PD position control with gravity compensation** — Mellinger, D. & Kumar, V. (2011). "Minimum snap trajectory generation and control for quadrotors." *IEEE ICRA*, pp. 2520–2525.
- **Consensus formation control** — Olfati-Saber, R. & Murray, R.M. (2004). "Consensus problems in networks of agents with switching topology and time-delays." *IEEE Transactions on Automatic Control* 49(9): 1520–1533. See also Olfati-Saber, R. (2006). "Flocking for multi-agent dynamic systems: algorithms and theory." *IEEE Transactions on Automatic Control* 51(3): 401–420.
- **Artificial potential field avoidance** — Khatib, O. (1986). "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots." *International Journal of Robotics Research* 5(1): 90–98.
- **Combined consensus formation with potential-field avoidance for quadrotors** — Kuriki, Y. & Namerikawa, T. (2014). "Consensus-based cooperative formation control with collision avoidance for a multi-UAV system." *2014 American Control Conference*, pp. 2077–2082.
- **Behavioural antecedent** — Reynolds, C.W. (1987). "Flocks, herds, and schools: A distributed behavioral model." *SIGGRAPH '87*.
