<p align="center">
  <img src="assets/amrita_logo.jpg"
       alt="Amrita Vishwa Vidyapeetham Logo"
       width="300">
</p>

<h1 align="center">Attitude Control of a Camera-Mounted Tethered Quadrotor for Infrastructure Inspection</h1>

<p align="center">
  <b>MATLAB/Simulink-Based Simulation of Tethered Quadrotor Attitude and Camera Stabilization</b><br>
  Amrita Vishwa Vidyapeetham
</p>

---

# Team Members

> Add your team members, roll numbers, and emails below.

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | Add Name | Add Roll Number | — |
| 2 | Add Name | Add Roll Number | — |
| 3 | Add Name | Add Roll Number | — |
| 4 | Add Name | Add Roll Number | — |
| 5 | Add Name | Add Roll Number | — |

---

# Abstract

Infrastructure inspection of tunnels, bridges, and similar structures can be difficult when close-range human inspection is restricted by limited space, traffic restrictions, or poor GPS availability. The referenced research proposes a tethered quadrotor carrying a camera so that the aircraft can be controlled without depending on GPS or continuous external joystick operation.

This project develops a MATLAB/Simulink simulation based on the attitude-control and camera-stabilization concepts presented in the reference paper, **"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection"** by Keigo Watanabe, Nao Moritoki, and Isaku Nagai.

The simulation contains three main parts: quadrotor roll/pitch attitude control, a 2-DOF camera stabilizer, and tether-based position sensing. The quadrotor attitude controller is implemented using PD control and is compared against P control under roll and pitch disturbances. The camera stabilizer uses PID control for roll and pitch so that the camera attitude remains close to 0 degrees while the supporting base is moved. The tether-position function calculates the quadrotor position from tether length and two tether inclination angles.

The implementation is intended as a simulation study of the control concepts in the paper. It does not claim to reproduce the complete physical quadrotor or every hardware dynamic of the original experiment.

---

# 1. Introduction

Quadrotors are useful for inspection tasks because they can perform vertical take-off and landing and can hover in locations where a conventional vehicle cannot operate. This makes them suitable for infrastructure inspection in environments such as tunnels and bridge structures.

However, direct operation of a quadrotor near an inspection target can require considerable operator skill. GPS-based position control can also become unreliable or unavailable inside tunnels or beneath bridge beams.

The reference paper addresses this problem using a **tethered quadrotor**. The tether provides information about the aircraft position through its inclination and length, reducing the dependence on GPS or external position information.

The paper also mounts a camera on the quadrotor. Because the aircraft attitude changes during flight, directly attaching the camera to the body would cause the camera field of view to change. Therefore, a **2-DOF camera stabilizer** is used to compensate for roll and pitch motion.

This project converts the main control concepts into a MATLAB/Simulink simulation so that the attitude controller, camera stabilizer, disturbances, and tether-position equations can be studied in a reproducible environment.

---

# 2. Problem Statement

The objective of this project is to simulate the control of a tethered camera-mounted quadrotor so that:

- the quadrotor can regulate its roll and pitch attitude,
- PD control can be evaluated against P control,
- disturbances can be applied to the attitude dynamics,
- the camera can maintain a stable roll and pitch orientation,
- tether inclination can be used to calculate the quadrotor position,
- and the simulation results can be compared conceptually with the experiments reported in the reference paper.

---

# 3. Objectives

The main objectives of the project are:

### 3.1 Quadrotor Attitude Control

Implement roll and pitch attitude controllers based on the PD equations given in the reference paper.

### 3.2 P vs PD Comparison

Simulate both P and PD control under external disturbances and compare the resulting attitude response.

### 3.3 Camera Stabilization

Implement a 2-DOF roll/pitch camera stabilizer using PID control.

### 3.4 Tether-Based Position Sensing

Implement the mathematical relationship between tether inclination, tether length, and quadrotor position.

### 3.5 Simulink Model

Construct a block-level simulation containing the quadrotor rotational dynamics, attitude controller, camera stabilizer, disturbances, and tether-position calculation.

### 3.6 Reproducible Simulation

Provide MATLAB scripts and a Simulink model that can be executed and modified for further controller studies.

---

# 4. Reference Paper

## Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection

**Authors:** Keigo Watanabe, Nao Moritoki, Isaku Nagai

**Affiliation:** Graduate School of Natural Science and Technology, Okayama University

The reference paper describes a tethered quadrotor intended as a backup system for infrastructure inspection. It presents:

- the structure and flight principle of a quadrotor,
- tether-based position detection,
- PD attitude control of the quadrotor,
- a 2-DOF camera stabilizer,
- PID control of the camera stabilizer,
- attitude-control experiments,
- and camera-stabilization experiments.

The paper changes the previous 3-DOF camera stabilizer into a 2-DOF mechanism because yaw motion can be handled by the main quadrotor body. The change also reduces weight and energy consumption and helps lower the center of gravity.

---

# 5. Project Scope

The simulation is organized into the following main components:

```text
                    TETHERED QUADROTOR SIMULATION
                               │
             ┌─────────────────┼─────────────────┐
             │                 │                 │
             ▼                 ▼                 ▼
      Attitude Control   Camera Stabilizer   Tether Position
             │                 │                 │
             ▼                 ▼                 ▼
          Roll/Pitch        Roll/Pitch       x, y, z
             │                 │
             ▼                 ▼
        P vs PD Control      PID Control
             │                 │
             └────────────┬────┘
                          ▼
                  MATLAB / Simulink
```

The current simulation focuses on rotational attitude dynamics and camera stabilization. The tether-position calculation is implemented as a sensing/position calculation and is not connected to a complete closed-loop position controller.

---

# 6. Methodology

## 6.1 Quadrotor Coordinate and Attitude Variables

The quadrotor attitude is represented by:

- $\phi$ — roll angle
- $\theta$ — pitch angle
- $\psi$ — yaw angle

The corresponding angular velocities are:

- $\dot{\phi}$
- $\dot{\theta}$
- $\dot{\psi}$

The reference paper defines the attitude variables in the quadrotor coordinate system and uses rotor thrust differences to control the body attitude.

In the present simulation, the rotational dynamics are simplified to independent roll, pitch, and yaw equations:

$$
\ddot{\phi} = \frac{U_2}{I_x}
$$

$$
\ddot{\theta} = \frac{U_3}{I_y}
$$

$$
\ddot{\psi} = \frac{U_4}{I_z}
$$

where $U_2$, $U_3$, and $U_4$ are the simulated attitude control inputs.

---

## 6.2 Quadrotor Physical Parameters

The simulation uses the measured principal moments of inertia reported in the paper:

| Parameter | Value |
|---|---:|
| $I_x$ | 0.01910 kg·m² |
| $I_y$ | 0.01910 kg·m² |
| $I_z$ | 0.03083 kg·m² |
| Tether length $l$ | 0.25 m |

The paper's Table I reports the measured values as 0.01910 kg·m², 0.01910 kg·m², and 0.03083 kg·m² for $I_x$, $I_y$, and $I_z$, respectively.

The `params.m` file additionally defines:

- mass $m = 1.2$ kg
- gravitational acceleration $g = 9.81$ m/s²

These values are provided as project parameters, although the present simplified rotational dynamics use the moments of inertia directly.

---

# 7. Quadrotor Attitude Controller

The reference paper uses a PD controller for quadrotor attitude control.

For roll:

$$
U_2=-K_1(\phi-\phi_d)-K_2\dot{\phi}
$$

For pitch:

$$
U_3=-K_3(\theta-\theta_d)-K_4\dot{\theta}
$$

For yaw, the paper gives:

$$
U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}
$$

The gains used for roll and pitch in the present simulation are:

| Gain | Value | Purpose |
|---|---:|---|
| $K_1$ | 0.625 | Roll proportional gain |
| $K_2$ | 0.170 | Roll derivative gain |
| $K_3$ | 0.810 | Pitch proportional gain |
| $K_4$ | 0.340 | Pitch derivative gain |

The simulation's Simulink model sets the yaw control input to zero, so yaw is not actively controlled in the current model.

---

# 8. P vs PD Attitude Simulation

The `main.m` script performs a direct numerical comparison between P and PD control.

### Roll P Controller

$$
U_{P,\phi}=-K_1\phi+d_\phi
$$

### Roll PD Controller

$$
U_{PD,\phi}=-K_1\phi-K_2\dot{\phi}+d_\phi
$$

Similarly, pitch is simulated using P and PD control.

The simulation uses:

- Time step: `0.01 s`
- Attitude simulation duration: `10 s`

The disturbances used in `main.m` are:

$$
d_\phi =
0.08\sin(2\pi(0.8)t)
+
0.05\sin(2\pi(0.3)t)
$$

and

$$
d_\theta =
0.09\sin(2\pi(0.75)t)
+
0.04\cos(2\pi(0.25)t)
$$

These disturbances are used to test the response of the P and PD controllers.

---

# 9. Camera Stabilizer

The reference paper originally used a 3-DOF stabilizer but changed it to a **2-DOF stabilizer**.

The two controlled camera axes are:

- Roll: $\phi_C$
- Pitch: $\theta_C$

Yaw is not independently controlled by the stabilizer because the paper assumes that yaw can be handled by the main quadrotor body.

The 2-DOF design provides the following advantages described in the paper:

- reduced weight,
- reduced energy consumption,
- lower center of gravity,
- simpler stabilization of the main body.

---

# 10. Camera PID Controller

The camera stabilizer uses PID control.

For camera roll:

$$
U_{\phi_C}
=
K_{11}e_{\phi_C}
+
K_{12}\int e_{\phi_C}dt
+
K_{13}\dot e_{\phi_C}
$$

For camera pitch:

$$
U_{\theta_C}
=
K_{14}e_{\theta_C}
+
K_{15}\int e_{\theta_C}dt
+
K_{16}\dot e_{\theta_C}
$$

The camera errors are:

$$
e_{\phi_C}=\phi_{Cd}-\phi_C
$$

$$
e_{\theta_C}=\theta_{Cd}-\theta_C
$$

The gains used are:

| Gain | Value |
|---|---:|
| $K_{11}$ | 170.0 |
| $K_{12}$ | 2.0 |
| $K_{13}$ | 0.1 |
| $K_{14}$ | 170.0 |
| $K_{15}$ | 2.0 |
| $K_{16}$ | 0.1 |

These are the same gain values reported for the camera-stabilizer experiment in the reference paper.

---

# 11. Camera Disturbance Simulation

To reproduce the type of camera-stabilization experiment described in the paper, the simulation applies a changing base/manipulator angle.

The motion is:

```text
0°  →  +45°  →  -45°  →  0°
```

The timing is:

| Time | Manipulator Angle |
|---:|---:|
| 0 s | 0° |
| 7 s | +45° |
| 21 s | -45° |
| 28 s | 0° |

The camera controller attempts to compensate for this base motion so that the net camera roll and pitch remain close to zero.

---

# 12. Simplified Camera Actuator Model

The MATLAB and Simulink implementations use a simplified actuator model to represent the camera servo response.

For the camera roll axis:

$$
\ddot{\phi}_C
=
100U_{\phi_C}
-
10\dot{\phi}_C
$$

A corresponding equation is used for camera pitch.

This is a **simulation model used in this project**. The reference paper specifies the PID controller and the physical servo hardware, but it does not provide this exact second-order servo dynamic equation.

Therefore, this actuator model should not be interpreted as the exact physical servo dynamics of the original hardware.

---

# 13. Tether-Based Position Sensing

One of the important concepts in the reference paper is estimating the quadrotor position from the tether.

Let:

- $l$ = tether length
- $\alpha$ = tether inclination toward the $E_x$ direction
- $\beta$ = tether inclination toward the $E_y$ direction
- $x,y,z$ = quadrotor position

The paper gives:

$$
x=z\tan\alpha
$$

$$
y=z\tan\beta
$$

and:

$$
z=
-\sqrt{
\frac{
l^2(\cos\alpha)^2(\cos\beta)^2
}{
(\cos\alpha)^2+(\cos\beta)^2
-(\cos\alpha)^2(\cos\beta)^2
}
}
$$

The implementation of these equations is contained in:

```text
tether_pos_func.m
```

---

# 14. Tether Position Example

The current MATLAB simulation uses:

```text
Tether length = 0.25 m
α = 15°
β = 10°
```

Using these values, the calculated position is approximately:

```text
x = -0.0638 m
y = -0.0420 m
z = -0.2381 m
```

The negative sign of $z$ follows the coordinate convention used by the equations in the reference paper.

The calculation demonstrates how tether inclination can provide the horizontal position components when tether length and inclination measurements are available.

---

# 15. MATLAB Files

The repository contains the following main files:

```text
drones project/
│
├── main.m
├── params.m
├── tether_pos_func.m
├── build_simulink_model.m
├── tethered_quadrotor_model.slx
└── tethered_quadrotor_model.slxc
```

### `main.m`

Runs the numerical MATLAB simulations for:

- roll P control,
- roll PD control,
- pitch P control,
- pitch PD control,
- camera roll stabilization,
- camera pitch stabilization,
- tether-position calculation.

It also generates the corresponding plots.

### `params.m`

Stores common physical and controller parameters, including:

- mass,
- gravity,
- tether length,
- moments of inertia,
- attitude gains,
- camera PID gains,
- desired attitude values.

### `tether_pos_func.m`

Implements the tether-based position equations and returns:

```text
[x; y; z]
```

from:

```text
[alpha; beta; l]
```

### `build_simulink_model.m`

Programmatically constructs the Simulink model and creates the main simulation subsystems.

### `tethered_quadrotor_model.slx`

The main Simulink model containing the simulation blocks.

---

# 16. Simulink Model Structure

The Simulink model contains the following major subsystems:

```text
tethered_quadrotor_model
│
├── Quadrotor_Attitude_Controller
│       ├── Roll control
│       └── Pitch control
│
├── Quadrotor_Rotational_Dynamics
│       ├── Roll dynamics
│       ├── Pitch dynamics
│       └── Yaw dynamics
│
├── Camera_Stabilizer_2DOF
│       ├── Camera roll PID
│       └── Camera pitch PID
│
├── Tether_Position_Sensing
│       ├── α input
│       ├── β input
│       └── x, y, z calculation
│
├── Roll disturbance
├── Pitch disturbance
├── Manipulator motion
└── Scopes / position display
```

The Simulink model uses an `ode45` solver and a stop time of `28 s`.

---

# 17. Simulation Workflow

The overall workflow is:

```text
                 START
                   │
                   ▼
          Load Model Parameters
                   │
                   ▼
       Define Quadrotor Inertia
                   │
                   ▼
          Apply Attitude Input
                   │
                   ▼
         P / PD Attitude Control
                   │
                   ▼
       Rotational Quadrotor Model
                   │
                   ▼
          Apply Disturbances
                   │
                   ▼
         Observe Roll / Pitch
                   │
                   ├─────────────────────┐
                   │                     │
                   ▼                     ▼
          Camera Base Motion       Tether Angles
                   │                     │
                   ▼                     ▼
            Camera PID             Position Sensing
                   │                     │
                   ▼                     ▼
        Camera Roll / Pitch          x, y, z
                   │                     │
                   └──────────┬──────────┘
                              ▼
                         RESULTS
```

---

# 18. Results

## 18.1 Quadrotor Attitude Control

The MATLAB simulation generates separate plots for:

- roll angle using P control,
- roll angle using PD control,
- pitch angle using P control,
- pitch angle using PD control.

The purpose is to observe the difference between proportional-only feedback and proportional-derivative feedback when disturbances are applied.

The reference paper experimentally reports that PD control produced smaller maximum attitude errors than P control:

| Attitude | P Control Maximum Error | PD Control Maximum Error |
|---|---:|---:|
| Roll | 12.0° | 8.7° |
| Pitch | −10.9° | 4.1° |

These values are **experimental results reported in the paper**, not values that should automatically be attributed to the MATLAB simulation.

---

## 18.2 Camera Stabilization

The camera simulation generates:

- camera roll angle vs manipulator angle,
- camera pitch angle vs manipulator angle.

The desired behavior is that the manipulator can move through large angles while the net camera attitude remains close to zero.

The reference paper reports that the physical camera stabilizer maintained both roll and pitch near 0° while the robot-arm tip was changed by ±45°.

---

## 18.3 Tether Position

The tether-position calculation demonstrates the relationship between:

```text
Tether length
      +
Tether inclination α
      +
Tether inclination β
      ↓
Quadrotor position (x, y, z)
```

For the current example:

```text
l = 0.25 m
α = 15°
β = 10°
```

the implementation produces approximately:

```text
x = -0.0638 m
y = -0.0420 m
z = -0.2381 m
```

---

# 19. Reference Paper vs Current Simulation

It is important to distinguish the physical research system from this simulation.

| Component | Reference Paper | Current Simulation |
|---|---|---|
| Quadrotor | Physical tethered quadrotor | Mathematical/Simulink rotational model |
| Roll control | PD | PD |
| Pitch control | PD | PD |
| Camera stabilizer | Physical 2-DOF mechanism | Simulated 2-DOF controller |
| Camera control | PID | PID |
| Tether sensing | Physical inclination sensor/potentiometer | Mathematical calculation |
| Position control | Described in paper | Not implemented as a complete closed loop |
| Yaw stabilizer | Main airframe handles yaw | $U_4$ set to zero in current Simulink model |
| Camera servo | Physical RC servos | Simplified actuator dynamics |
| Experiment | Physical hardware experiments | MATLAB/Simulink simulation |

This distinction is important when interpreting the results.

---

# 20. Installation Requirements

The project requires:

- MATLAB
- Simulink

The project was designed around standard MATLAB/Simulink blocks and does not require Python for the main simulation.

---

# 21. How to Run

## Step 1 — Open MATLAB

Start MATLAB and navigate to the project folder.

## Step 2 — Add the Project Folder to the MATLAB Path

Make sure the folder containing the following files is the current MATLAB working directory:

```text
main.m
params.m
tether_pos_func.m
build_simulink_model.m
tethered_quadrotor_model.slx
```

## Step 3 — Run the MATLAB Simulation

Run:

```matlab
main
```

This generates the attitude-control and camera-stabilization plots.

## Step 4 — Open the Simulink Model

Open:

```matlab
open_system('tethered_quadrotor_model')
```

## Step 5 — Rebuild the Simulink Model if Required

Run:

```matlab
build_simulink_model
```

This programmatically creates/rebuilds the Simulink model.

## Step 6 — Run the Simulink Simulation

Run the model from the Simulink interface.

The model is configured with:

```text
Solver  : ode45
StopTime: 28 s
```

---

# 22. Expected Outputs

Running `main.m` produces three main figures.

### Figure 1 — Roll Attitude

```text
P Control
     vs
PD Control
```

### Figure 2 — Pitch Attitude

```text
P Control
     vs
PD Control
```

### Figure 3 — Camera Stabilization

```text
Camera Roll  + Manipulator Angle
Camera Pitch + Manipulator Angle
```

The Simulink model additionally provides scopes for:

- quadrotor attitude,
- camera roll,
- camera pitch,

and a display for tether-based position.

---

# 23. Limitations

The current implementation has several important modeling limitations.

### 23.1 Simplified Rotational Dynamics

The quadrotor dynamics are represented by independent second-order rotational equations based on moment of inertia. Full nonlinear rigid-body dynamics, rotor thrust coupling, aerodynamic effects, and gyroscopic effects are not modeled.

### 23.2 Simplified Camera Servo Model

The camera actuator is represented using a simplified second-order dynamic model. The exact electrical and mechanical dynamics of the physical SAVOX servos are not modeled.

### 23.3 No Complete Closed-Loop Position Controller

The tether-position equations are implemented, but the current simulation does not use the calculated $x,y,z$ values in a complete autonomous position-control loop.

### 23.4 Yaw Control

Although the paper defines a yaw PD controller, the current Simulink model sets the yaw control input $U_4$ to zero.

### 23.5 Physical Tether Dynamics

The actual tether is not modeled as a flexible cable with mass, elasticity, tension, and aerodynamic effects. The simulation only implements the geometric tether-position relationship.

---

# 24. Future Work

Future improvements can extend the current simulation toward a more complete representation of the research system.

Possible extensions include:

- Implement full nonlinear quadrotor dynamics.
- Add complete yaw control.
- Connect tether-based position estimation to a closed-loop position controller.
- Model tether tension and flexible-tether dynamics.
- Add realistic motor and propeller models.
- Add physical servo dynamics for the camera stabilizer.
- Include sensor noise and measurement uncertainty.
- Implement a complete autonomous position-control system.
- Extend the simulation to infrastructure-inspection trajectories.
- Validate the controller using experimental or hardware-in-the-loop data.

The reference paper itself identifies future work involving yaw control, position control, and establishing and adjusting an autonomous control system as a complete system.

---

# 25. Project Contributions

This project provides a reproducible simulation framework for studying:

- Tethered quadrotor attitude control
- P vs PD controller behavior
- 2-DOF camera stabilization
- PID-based camera control
- Tether-based position estimation
- MATLAB numerical simulation
- Simulink block-level modeling
- Controller parameter experimentation

---

# 26. Repository Structure

```text
.
├── README.md
│
├── main.m
├── params.m
├── tether_pos_func.m
├── build_simulink_model.m
│
├── tethered_quadrotor_model.slx
├── tethered_quadrotor_model.slxc
│
└── assets/
    └── amrita_logo.jpg
```

---

# 27. Acknowledgement

This project is based on the control concepts and experimental work presented in:

**K. Watanabe, N. Moritoki, and I. Nagai, "Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection."**

The original work investigates a camera-mounted tethered quadrotor for infrastructure inspection and demonstrates quadrotor attitude control and 2-DOF camera stabilization through physical experiments.

---

# 28. License

This repository is intended for academic and educational use.

The reference paper and its original content remain the property of their respective authors and publisher. This repository contains an independent MATLAB/Simulink implementation for academic study.

---

# 29. Keywords

```text
Tethered Quadrotor
Quadrotor Control
Attitude Control
PD Controller
PID Controller
Camera Stabilization
2-DOF Stabilizer
Tether Position Sensing
MATLAB
Simulink
Infrastructure Inspection
UAV
Drone Simulation
Roll Control
Pitch Control
```
