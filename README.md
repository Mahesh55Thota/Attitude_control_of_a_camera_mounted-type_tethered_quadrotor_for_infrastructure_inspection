<p align="center">
  <img src="assets/amrita_logo.jpg"
       alt="Amrita Vishwa Vidyapeetham Logo"
       width="300">
</p>

<h1 align="center">Attitude Control of a Camera-Mounted Tethered Quadrotor for Infrastructure Inspection</h1>

<p align="center">
  <b>MATLAB/Simulink-Based Simulation of Tethered Quadrotor Attitude Control and Camera Stabilization</b><br>
  Amrita Vishwa Vidyapeetham
</p>

---

# Team Members

> Replace the following placeholders with your actual team information.

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | Add Name | Add Roll Number | — |
| 2 | Add Name | Add Roll Number | — |
| 3 | Add Name | Add Roll Number | — |
| 4 | Add Name | Add Roll Number | — |
| 5 | Add Name | Add Roll Number | — |

---

# Abstract

Infrastructure inspection of tunnels, bridges, and similar structures can become difficult when direct human inspection is restricted by limited working space, traffic restrictions, or inaccessible locations. Unmanned aerial vehicles can provide an alternative inspection platform because quadrotors can perform vertical take-off, landing, and hovering.

The reference research proposes a tethered quadrotor equipped with a camera for infrastructure inspection. The tether provides position information from its length and inclination, reducing dependence on GPS or continuous external operation. The research also introduces a camera stabilizer so that the camera field of view remains stable even when the quadrotor body changes its attitude.

This project develops a MATLAB and Simulink simulation based on the main control concepts presented in the reference paper, **"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection"** by Keigo Watanabe, Nao Moritoki, and Isaku Nagai.

The project contains three major simulation components: quadrotor attitude control, 2-DOF camera stabilization, and tether-based position calculation. The quadrotor roll and pitch dynamics are modeled using simplified rotational equations and controlled using PD control. The camera stabilizer uses PID control for roll and pitch. The tether-position function calculates the quadrotor position from tether length and two tether inclination angles.

The project provides both a MATLAB numerical simulation and a MATLAB/Simulink implementation for studying the control behavior of the proposed system.

---

# 1. Introduction

Quadrotors are increasingly used for inspection applications because they can hover and operate in locations where conventional inspection equipment may not be suitable.

Infrastructure inspection of tunnels and bridges can be particularly challenging. Human proximity inspection may become difficult when there is insufficient space for height-working equipment or when large-scale traffic restrictions would be required.

A quadrotor can approach an inspection target and carry a camera for visual observation. However, conventional quadrotor operation generally depends on external operator commands or position information such as GPS.

GPS can become unreliable or unavailable inside tunnels or underneath bridge structures. To address this problem, the reference research uses a **tethered quadrotor**. The tether is attached to the quadrotor and its inclination is measured. Together with the known tether length, the inclination measurements can be used to estimate the position of the aircraft.

Another challenge occurs when the camera is directly mounted on the quadrotor. Any roll or pitch motion of the aircraft changes the camera field of view. Therefore, the reference research uses a camera stabilizer that compensates for roll and pitch motion.

This project implements the major control concepts of that system in MATLAB and Simulink.

---

# 2. Problem Statement

The objective of this project is to simulate a tethered quadrotor equipped with a camera and investigate the control mechanisms required to maintain stable quadrotor attitude and camera orientation.

The simulation should:

- Model the quadrotor roll, pitch, and yaw rotational dynamics.
- Implement PD-based roll and pitch attitude control.
- Apply disturbances to the roll and pitch dynamics.
- Implement a 2-DOF camera stabilizer.
- Use PID control for camera roll and pitch stabilization.
- Apply a changing manipulator/base angle to test camera stabilization.
- Calculate quadrotor position from tether length and inclination angles.
- Provide MATLAB plots and a Simulink model for analyzing the system.

---

# 3. Objectives

The main objectives of the project are:

### 3.1 Quadrotor Attitude Control

Implement PD controllers for the roll and pitch angles of the quadrotor.

### 3.2 Disturbance Rejection

Apply external disturbances to the roll and pitch dynamics and observe the controller response.

### 3.3 Camera Stabilization

Develop a 2-DOF camera stabilization system for roll and pitch.

### 3.4 PID Control

Implement proportional, integral, and derivative control for the camera stabilizer.

### 3.5 Tether-Based Position Calculation

Calculate the quadrotor position using tether length and tether inclination angles.

### 3.6 MATLAB Simulation

Perform numerical simulations of quadrotor attitude and camera stabilization.

### 3.7 Simulink Implementation

Develop a block-based Simulink model containing the quadrotor dynamics, attitude controller, camera stabilizer, disturbances, and tether-position calculation.

---

# 4. Reference Paper

## Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection

The project is based on the research paper:

**K. Watanabe, N. Moritoki, and I. Nagai,  
"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection."**

The paper investigates a tethered quadrotor carrying a camera for infrastructure inspection.

The main topics presented in the paper are:

- Quadrotor structure and flight principle
- Tether-based position detection
- Quadrotor attitude control
- PD control of the quadrotor
- 2-DOF camera stabilizer
- PID control of the camera stabilizer
- Roll and pitch attitude experiments
- Camera stabilization experiments

The paper explains that the previous camera stabilizer used 3 DOFs. In the presented system, yaw stabilization is removed from the camera mechanism because yaw motion can be handled by the main quadrotor body. This results in a 2-DOF stabilizer consisting of roll and pitch motion.

The paper reports that removing the yaw degree of freedom provides benefits including reduced weight and energy consumption and a lower center of gravity. :contentReference[oaicite:0]{index=0}

---

# 5. Project Scope

The current project focuses on simulation rather than a complete physical quadrotor implementation.

The overall simulation consists of:

```text
                 TETHERED QUADROTOR
                         |
          +--------------+--------------+
          |              |              |
          v              v              v
   Attitude Control   Camera       Tether Position
      System        Stabilizer        Sensing
          |              |              |
          v              v              v
      Roll/Pitch     Roll/Pitch       x, y, z
          |              |
          v              v
         PD             PID
          |              |
          +-------+------+ 
                  |
                  v
           MATLAB / Simulink
```

The current implementation does not model a complete nonlinear quadrotor flight system. It focuses mainly on rotational attitude dynamics, camera stabilization, and tether-position geometry.

---

# 6. System Architecture

The MATLAB/Simulink project contains the following main components:

```text
                    tethered_quadrotor_model
                              |
          +-------------------+-------------------+
          |                   |                   |
          v                   v                   v
 Quadrotor Attitude    Camera Stabilizer    Tether Position
    Controller             2-DOF                Sensing
          |                   |                   |
          v                   v                   v
       U2, U3          Camera Roll/Pitch         x,y,z
          |                   ^
          v                   |
 Quadrotor Rotational   Manipulator Motion
      Dynamics
          |
          v
   phi, theta, psi
```

---

# 7. Quadrotor Structure

The reference paper considers a quadrotor with four rotors arranged symmetrically around the body.

The four rotors generate thrust by changing their rotational speeds. The quadrotor attitude is represented by:

- $\phi$ — roll angle
- $\theta$ — pitch angle
- $\psi$ — yaw angle

The corresponding angular velocities are:

- $\dot{\phi}$
- $\dot{\theta}$
- $\dot{\psi}$

The reference paper explains that the quadrotor can control position and attitude by changing the rotational speeds of the four rotors. Equal rotor speeds are used for hovering, while opposite rotor directions help cancel the reaction torque around the vertical axis. :contentReference[oaicite:1]{index=1}

---

# 8. Quadrotor Parameters

The project uses the measured principal moments of inertia reported in the reference paper.

| Parameter | Value |
|---|---:|
| $I_x$ | 0.01910 kg·m² |
| $I_y$ | 0.01910 kg·m² |
| $I_z$ | 0.03083 kg·m² |
| Tether length $l$ | 0.25 m |

The project parameter file additionally contains:

| Parameter | Value |
|---|---:|
| Mass $m$ | 1.2 kg |
| Gravity $g$ | 9.81 m/s² |

The mass and gravity values are defined in `params.m`, but the current simplified rotational equations do not directly use them.

The measured inertia values in the reference paper are listed in Table I. :contentReference[oaicite:2]{index=2}

---

# 9. Quadrotor Rotational Dynamics

The Simulink model represents the rotational dynamics using simplified second-order equations.

For roll:

$$\ddot{\phi}=\frac{U_2}{I_x}$$

For pitch:

$$\ddot{\theta}=\frac{U_3}{I_y}$$

For yaw:

$$\ddot{\psi}=\frac{U_4}{I_z}$$

where:

- $U_2$ is the roll control input.
- $U_3$ is the pitch control input.
- $U_4$ is the yaw control input.
- $I_x$, $I_y$, and $I_z$ are the principal moments of inertia.

The Simulink implementation realizes these equations using a gain of $1/I$ followed by two integrators.

Therefore, the model represents:

```text
Control Input
     |
     v
   1 / I
     |
     v
Angular Acceleration
     |
     v
  Integrator
     |
     v
Angular Velocity
     |
     v
  Integrator
     |
     v
Angular Position
```

---

# 10. Quadrotor Attitude Controller

The reference paper uses PD control for quadrotor attitude.

For roll:

$$U_2=-K_1(\phi-\phi_d)-K_2\dot{\phi}$$

For pitch:

$$U_3=-K_3(\theta-\theta_d)-K_4\dot{\theta}$$

For yaw, the paper defines:

$$U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}$$

The current project implements the roll and pitch controllers in Simulink.

The attitude gains used in the project are:

| Gain | Value | Purpose |
|---|---:|---|
| $K_1$ | 0.625 | Roll proportional gain |
| $K_2$ | 0.170 | Roll derivative gain |
| $K_3$ | 0.810 | Pitch proportional gain |
| $K_4$ | 0.340 | Pitch derivative gain |
| $K_5$ | 0.370 | Yaw proportional gain parameter |
| $K_6$ | 0.100 | Yaw derivative gain parameter |

The current Simulink model does **not** implement the yaw PD controller. Instead, the yaw control input is supplied as a constant zero signal.

The roll and pitch PD controller equations are directly based on the controller equations presented in the paper. :contentReference[oaicite:3]{index=3}

---

# 11. Desired Attitude

The current project uses zero desired roll and pitch:

$$\phi_d=0$$

$$\theta_d=0$$

The purpose of the controller is therefore to bring the roll and pitch angles back toward zero when disturbances are applied.

In the Simulink model:

```text
phi_d   = 0 rad
theta_d = 0 rad
```

The yaw reference is also defined in `params.m`:

```text
psi_d = 0
```

but the current Simulink model does not actively use the yaw PD controller.

---

# 12. Roll and Pitch Disturbances

The MATLAB simulation introduces disturbances into the roll and pitch dynamics.

The roll disturbance is:

$$d_{\phi}(t)=0.08\sin(2\pi(0.8)t)+0.05\sin(2\pi(0.3)t)$$

The pitch disturbance is:

$$d_{\theta}(t)=0.09\sin(2\pi(0.75)t)+0.04\cos(2\pi(0.25)t)$$

These disturbances are added to the controller output before the rotational dynamics.

The MATLAB simulation uses:

```text
Time step = 0.01 s
Attitude simulation time = 10 s
```

The purpose is to test how the P and PD controllers respond to changing disturbances.

---

# 13. P vs PD Attitude Simulation

The `main.m` file compares P and PD control for both roll and pitch.

## Roll P Control

The proportional controller is:

$$U_{P,\phi}=-K_1\phi+d_{\phi}$$

## Roll PD Control

The proportional-derivative controller is:

$$U_{PD,\phi}=-K_1\phi-K_2\dot{\phi}+d_{\phi}$$

## Pitch P Control

$$U_{P,\theta}=-K_3\theta+d_{\theta}$$

## Pitch PD Control

$$U_{PD,\theta}=-K_3\theta-K_4\dot{\theta}+d_{\theta}$$

The simulation produces separate plots for:

- Roll angle using P control
- Roll angle using PD control
- Pitch angle using P control
- Pitch angle using PD control

The purpose of the comparison is to observe the effect of the derivative term on the attitude response.

---

# 14. Reference Paper Experimental Results

The reference paper experimentally compares P and PD control for roll and pitch.

For roll:

| Controller | Maximum Error |
|---|---:|
| P control | 12.0° |
| PD control | 8.7° |

For pitch:

| Controller | Maximum Error |
|---|---:|
| P control | −10.9° |
| PD control | 4.1° |

The paper concludes that PD control produced better attitude-error performance than P control for both roll and pitch. :contentReference[oaicite:4]{index=4}

**Important:** These values are the experimental results reported by the paper. They should not be described as the numerical results of the MATLAB simulation unless the simulation produces the same values.

---

# 15. Camera Stabilizer

The camera is stabilized using a 2-DOF mechanism.

The two camera degrees of freedom are:

- Camera roll: $\phi_C$
- Camera pitch: $\theta_C$

The reference system previously used a 3-DOF camera stabilizer. The yaw degree of freedom was removed because yaw motion could be handled by the main quadrotor body.

The resulting 2-DOF mechanism provides:

- Roll stabilization
- Pitch stabilization

The paper states that removing one degree of freedom also helps reduce weight and energy consumption and lowers the center of gravity. :contentReference[oaicite:5]{index=5}

---

# 16. Camera Stabilizer Structure

The physical camera stabilizer described in the paper uses:

- Two RC servo motors
- An inertia sensor
- A camera mounting structure

The paper identifies the camera as an HDR-AZ1 camera and the inertia sensor as an MPU-9150. The stabilizer frame was manufactured using a 3D printer.

The physical stabilizer dimensions reported in the paper are:

| Dimension | Value |
|---|---:|
| Width | 75 mm |
| Depth | 140 mm |
| Height | 87 mm |

These hardware details belong to the reference physical system. The present project simulates the control behavior rather than reproducing the physical hardware in full. :contentReference[oaicite:6]{index=6}

---

# 17. Camera PID Controller

The reference paper uses PID control for the camera stabilizer.

For camera roll:

$$U_{\phi_C}=K_{11}e_{\phi_C}+K_{12}\int e_{\phi_C}dt+K_{13}\dot{e}_{\phi_C}$$

For camera pitch:

$$U_{\theta_C}=K_{14}e_{\theta_C}+K_{15}\int e_{\theta_C}dt+K_{16}\dot{e}_{\theta_C}$$

The camera errors are defined as:

$$e_{\phi_C}=\phi_{Cd}-\phi_C$$

$$e_{\theta_C}=\theta_{Cd}-\theta_C$$

The reference paper gives the PID controller structure and these error definitions. :contentReference[oaicite:7]{index=7}

---

# 18. Camera PID Gains

The project uses the following camera PID gains:

| Gain | Value | Purpose |
|---|---:|---|
| $K_{11}$ | 170.0 | Camera roll proportional gain |
| $K_{12}$ | 2.0 | Camera roll integral gain |
| $K_{13}$ | 0.1 | Camera roll derivative gain |
| $K_{14}$ | 170.0 | Camera pitch proportional gain |
| $K_{15}$ | 2.0 | Camera pitch integral gain |
| $K_{16}$ | 0.1 | Camera pitch derivative gain |

These values correspond to the gains reported for the camera-stabilizer experiment in the reference paper. :contentReference[oaicite:8]{index=8}

---

# 19. Camera Stabilizer Simulation

The project applies a changing base/manipulator angle to test whether the camera can maintain a stable orientation.

The manipulator motion is:

```text
0°  →  +45°  →  -45°  →  0°
```

The motion is defined over 28 seconds:

| Time | Manipulator Angle |
|---:|---:|
| 0 s | 0° |
| 7 s | +45° |
| 21 s | −45° |
| 28 s | 0° |

The same motion is applied to both the camera roll and pitch base inputs in the current simulation.

The reference paper performs a physical experiment in which the robot-arm tip is inclined by ±45° in the roll and pitch directions and checks whether the camera can remain at 0°. :contentReference[oaicite:9]{index=9}

---

# 20. Camera Stabilization Error

The camera controller uses the net camera orientation as its feedback quantity.

The simulated net camera roll is:

$$\phi_{camera}=\phi_{servo}+\phi_{base}$$

The simulated net camera pitch is:

$$\theta_{camera}=\theta_{servo}+\theta_{base}$$

The controller attempts to drive these net angles toward zero.

Therefore, if the base rotates in one direction, the servo should rotate the camera in the opposite direction.

Conceptually:

```text
Base Motion
     |
     v
Camera Base Angle
     |
     v
PID Controller
     |
     v
Servo Motion
     |
     v
Opposite Camera Compensation
     |
     v
Net Camera Angle → approximately 0°
```

---

# 21. Camera Actuator Model in MATLAB

The `main.m` MATLAB simulation uses a simplified second-order actuator model.

For camera roll:

$$\ddot{\phi}_{servo}=100U_{\phi_C}-10\dot{\phi}_{servo}$$

For camera pitch:

$$\ddot{\theta}_{servo}=100U_{\theta_C}-10\dot{\theta}_{servo}$$

The `100` term represents the actuator gain used in the simulation, while the `-10` velocity term provides damping in the MATLAB numerical model.

**This equation is part of the project's MATLAB simulation model. It is not an equation explicitly provided by the reference paper.**

---

# 22. Camera Actuator Model in Simulink

The Simulink implementation generated by `build_simulink_model.m` uses a slightly different simplified actuator structure.

The camera PID output is passed through a gain of:

```text
100
```

and then through two integrators:

```text
PID Output
    |
    v
 Gain = 100
    |
    v
Integrator
    |
    v
Servo Position
```

Therefore, the Simulink actuator structure corresponds to:

$$\ddot{\phi}_{servo}=100U_{\phi_C}$$

and similarly:

$$\ddot{\theta}_{servo}=100U_{\theta_C}$$

The Simulink model does **not** contain the `-10 × angular velocity` damping term used in `main.m`.

This distinction is intentionally documented here so that the MATLAB and Simulink implementations are not incorrectly described as mathematically identical.

---

# 23. Tether-Based Position Detection

The reference paper uses the tether inclination to determine the quadrotor position.

Let:

- $l$ = tether length
- $\alpha$ = tether inclination toward the $E_x$ direction
- $\beta$ = tether inclination toward the $E_y$ direction
- $x$ = position along the $E_x$ direction
- $y$ = position along the $E_y$ direction
- $z$ = vertical position

The paper gives:

$$x=z\tan\alpha$$

$$y=z\tan\beta$$

The vertical coordinate is calculated as:

$$z=-\sqrt{\frac{l^2(\cos\alpha)^2(\cos\beta)^2}{(\cos\alpha)^2+(\cos\beta)^2-(\cos\alpha)^2(\cos\beta)^2}}$$

These equations are implemented in `tether_pos_func.m`. :contentReference[oaicite:10]{index=10}

---

# 24. Tether Position Function

The MATLAB function:

```text
tether_pos_func.m
```

accepts:

```text
u(1) = alpha
u(2) = beta
u(3) = l
```

and returns:

```text
pos = [x; y; z]
```

The implementation first calculates:

```text
ca = cos(alpha)
cb = cos(beta)
```

Then:

```text
num = l² × ca² × cb²

den = ca² + cb² − ca² × cb²
```

The vertical position is calculated as:

$$z=-\sqrt{\frac{num}{den}}$$

when the denominator is positive.

If the denominator is not positive, the function uses:

```text
z = -l
```

as a fallback.

Finally:

$$x=z\tan\alpha$$

$$y=z\tan\beta$$

and the function returns:

```text
[x; y; z]
```

---

# 25. Tether Position Example

The current simulation uses:

```text
Tether length = 0.25 m
Alpha         = 15°
Beta          = 10°
```

Using the implemented equations, the approximate calculated position is:

```text
x = -0.06379 m
y = -0.04198 m
z = -0.23805 m
```

Therefore:

```text
Position ≈ [-0.06379 ; -0.04198 ; -0.23805] m
```

The negative sign of the vertical coordinate follows the coordinate convention used by the tether-position equation implemented in the project.

---

# 26. Tether Position in Simulink

The Simulink subsystem:

```text
Tether_Position_Sensing
```

contains:

```text
alpha
beta
  |
  +---- Constant l = 0.25
  |
  v
Mux
  |
  v
tether_pos_func
  |
  v
Demux
  |
  +---- x
  +---- y
  +---- z
```

The current Simulink model uses constant reference values:

```text
alpha = 15°
beta  = 10°
l     = 0.25 m
```

The calculated position is displayed using the `Display_Tether_Pos` block.

---

# 27. Simulink Model

The main Simulink model is:

```text
tethered_quadrotor_model.slx
```

The model contains the following major subsystems:

```text
tethered_quadrotor_model
│
├── Quadrotor_Attitude_Controller
│
├── Quadrotor_Rotational_Dynamics
│
├── Camera_Stabilizer_2DOF
│
├── Tether_Position_Sensing
│
├── Roll_Disturbance
│
├── Pitch_Disturbance
│
├── Manipulator_Motion_Disturb
│
├── Scope_Quadrotor_Attitude
│
├── Camera_Roll_Scope
│
├── Camera_Pitch_Scope
│
└── Display_Tether_Pos
```

---

# 28. Quadrotor Attitude Controller Subsystem

The Simulink subsystem:

```text
Quadrotor_Attitude_Controller
```

contains separate roll and pitch PD controllers.

## Roll

```text
phi_d
  |
  v
phi_d - phi
  |
  +---- K1 = -0.625 ----+
  |                     |
  |                 +---+---+
  |                 | Sum U2|
  |                 +---+---+
  |                     |
phidot                 U2
  |
K2 = -0.170
  |
  +---------------------+
```

## Pitch

```text
theta_d
   |
   v
theta_d - theta
   |
   +---- K3 = -0.810 ----+
   |                     |
   |                 +---+---+
   |                 | Sum U3|
   |                 +---+---+
   |                     |
thetadot                U3
   |
K4 = -0.340
   |
   +---------------------+
```

---

# 29. Quadrotor Rotational Dynamics Subsystem

The subsystem:

```text
Quadrotor_Rotational_Dynamics
```

contains three rotational channels.

### Roll Channel

```text
U2
 |
 v
1 / Ix
 |
 v
Integrator
 |
 v
phidot
 |
 v
Integrator
 |
 v
phi
```

### Pitch Channel

```text
U3
 |
 v
1 / Iy
 |
 v
Integrator
 |
 v
thetadot
 |
 v
Integrator
 |
 v
theta
```

### Yaw Channel

```text
U4
 |
 v
1 / Iz
 |
 v
Integrator
 |
 v
psidot
 |
 v
Integrator
 |
 v
psi
```

The yaw input in the current top-level model is supplied by:

```text
Yaw_U4_Zero
```

with value:

```text
0
```

---

# 30. Camera Stabilizer Subsystem

The Simulink subsystem:

```text
Camera_Stabilizer_2DOF
```

contains two independent PID control channels.

### Roll Channel

```text
Base Roll + Servo Roll
          |
          v
       Error
          |
          +---- K11 = -170
          |
          +---- Integral → K12 = -2
          |
          +---- Derivative → K13 = -0.1
          |
          v
       PID Sum
          |
          v
      Gain = 100
          |
          v
     Integrator
          |
          v
     Integrator
          |
          v
    Servo Roll
          |
          +------ Base Roll
                     |
                     v
               Camera Roll
```

### Pitch Channel

The pitch channel follows the same structure using:

```text
K14 = -170
K15 = -2
K16 = -0.1
```

and:

```text
Gain = 100
```

---

# 31. Manipulator Motion

The Simulink model uses a repeating sequence for the manipulator/base disturbance.

The sequence is:

```text
Time:  0     7     21     28 seconds

Angle: 0°   +45°   -45°    0°
```

The MATLAB implementation generates the same piecewise-linear motion.

The reference paper uses a physical robot manipulator and changes its tip inclination by ±45° to test the camera stabilizer. :contentReference[oaicite:11]{index=11}

---

# 32. Simulation Time

The project uses two simulation durations.

### MATLAB Attitude Simulation

```text
Time = 0 to 10 s
Step = 0.01 s
```

### MATLAB Camera Simulation

```text
Time = 0 to 28 s
Step = 0.01 s
```

### Simulink

The generated Simulink model is configured as:

```text
Solver   = ode45
Start    = 0 s
Stop     = 28 s
```

The Simulink solver configuration is intended to simulate the complete 28-second model.

---

# 33. MATLAB Files

The project contains four main MATLAB source files.

```text
main.m
params.m
tether_pos_func.m
build_simulink_model.m
```

---

# 34. `main.m`

The file:

```text
main.m
```

performs the main MATLAB numerical simulations.

It contains:

### Quadrotor parameters

```text
Ix
Iy
Iz
```

### Attitude gains

```text
K1
K2
K3
K4
```

### Camera PID gains

```text
K11
K12
K13
K14
K15
K16
```

### Tether parameters

```text
l
alpha
beta
```

### Attitude simulation

The script simulates:

```text
Roll P
Roll PD
Pitch P
Pitch PD
```

### Camera simulation

The script simulates:

```text
Camera Roll
Camera Pitch
```

under the changing manipulator angle.

### Plot generation

The script creates:

```text
Figure 1 → Roll P and PD
Figure 2 → Pitch P and PD
Figure 3 → Camera Roll and Pitch
```

---

# 35. `params.m`

The file:

```text
params.m
```

stores the main project parameters.

It defines:

```text
m  = 1.2
g  = 9.81
l  = 0.25

Ix = 0.01910
Iy = 0.01910
Iz = 0.03083
```

It also defines the quadrotor attitude gains:

```text
K1 = 0.625
K2 = 0.170
K3 = 0.810
K4 = 0.340
K5 = 0.370
K6 = 0.100
```

and camera gains:

```text
K11 = 170.0
K12 = 2.0
K13 = 0.1

K14 = 170.0
K15 = 2.0
K16 = 0.1
```

The desired angles are:

```text
phi_d   = 0
theta_d = 0
psi_d   = 0
```

---

# 36. `tether_pos_func.m`

The file:

```text
tether_pos_func.m
```

implements the tether-position calculation.

Input:

```text
u = [alpha; beta; l]
```

Output:

```text
pos = [x; y; z]
```

The function is used both independently from MATLAB and inside the Simulink tether-position subsystem.

---

# 37. `build_simulink_model.m`

The file:

```text
build_simulink_model.m
```

programmatically creates the Simulink model.

It:

1. Creates the model.
2. Sets the solver to `ode45`.
3. Sets the simulation stop time to 28 seconds.
4. Creates the quadrotor rotational dynamics subsystem.
5. Creates the quadrotor attitude controller subsystem.
6. Creates the 2-DOF camera stabilizer.
7. Creates the tether-position subsystem.
8. Adds roll and pitch disturbances.
9. Adds manipulator motion.
10. Adds scopes and displays.
11. Connects all the blocks.
12. Saves the model as:

```text
tethered_quadrotor_model.slx
```

---

# 38. Complete Simulation Workflow

The complete project workflow is:

```text
                 START
                   |
                   v
          Load Project Parameters
                   |
                   v
        Define Quadrotor Inertia
                   |
                   v
       Define Controller Gains
                   |
          +--------+--------+
          |                 |
          v                 v
   Attitude Simulation   Tether Geometry
          |                 |
          v                 v
     Roll / Pitch         x, y, z
          |
          v
    P and PD Control
          |
          v
      Disturbances
          |
          v
   Rotational Dynamics
          |
          v
   Roll / Pitch Response
          |
          +---------------------+
                                |
                                v
                       Camera Stabilizer
                                |
                                v
                         PID Controller
                                |
                                v
                        Servo Dynamics
                                |
                                v
                       Camera Orientation
                                |
                                v
                             RESULTS
```

---

# 39. Control Loop

The attitude control loop is:

```text
Desired Attitude
      |
      v
    Error
      |
      v
   PD Controller
      |
      v
 Control Input U2/U3
      |
      v
 Rotational Dynamics
      |
      v
 Current Attitude
      |
      +---------------------> Feedback
```

The camera stabilization loop is:

```text
Desired Camera Angle
        |
        v
     Camera Error
        |
        v
     PID Controller
        |
        v
     Servo Input
        |
        v
   Servo Dynamics
        |
        v
   Camera Orientation
        |
        +-------------------> Feedback
```

---

# 40. Expected MATLAB Outputs

Running:

```matlab
main
```

produces three figures.

## Figure 1 — Roll Attitude

The first figure contains:

```text
(a) Attitude control of the roll angle using P control

(b) Attitude control of the roll angle using PD control
```

The vertical axis represents:

```text
Roll angle [deg]
```

and the horizontal axis represents:

```text
Time [s]
```

---

## Figure 2 — Pitch Attitude

The second figure contains:

```text
(a) Attitude control of the pitch angle using P control

(b) Attitude control of the pitch angle using PD control
```

The vertical axis represents:

```text
Pitch angle [deg]
```

and the horizontal axis represents:

```text
Time [s]
```

---

## Figure 3 — Camera Stabilization

The third figure contains:

```text
(a) Roll angle of the camera

(b) Pitch angle of the camera
```

Each plot compares:

```text
Camera angle
```

with:

```text
Manipulator angle
```

The desired camera behavior is to remain close to zero while the manipulator angle changes.

---

# 41. Expected Simulink Outputs

The Simulink model contains:

### Quadrotor Attitude Scope

Displays:

```text
Roll angle
Pitch angle
```

in degrees.

### Camera Roll Scope

Displays:

```text
Camera roll angle
Manipulator angle
```

in degrees.

### Camera Pitch Scope

Displays:

```text
Camera pitch angle
Manipulator angle
```

in degrees.

### Tether Position Display

Displays:

```text
x
y
z
```

calculated from the tether geometry.

---

# 42. Reference Experimental Results

The reference paper reports two main physical experiments.

## Quadrotor Attitude Experiment

The quadrotor was suspended using strings and tested for roll and pitch attitude control.

The paper compares P and PD controllers.

For roll:

```text
P control  → maximum error = 12.0°
PD control → maximum error = 8.7°
```

For pitch:

```text
P control  → maximum error = −10.9°
PD control → maximum error = 4.1°
```

The paper concludes that PD control outperformed P control in terms of attitude error. :contentReference[oaicite:12]{index=12}

---

# 43. Reference Camera Stabilizer Experiment

The physical camera stabilizer was attached to a robot manipulator.

The robot arm was moved by ±45° in the roll and pitch directions.

The experiment checked whether the camera could maintain an approximately 0° orientation.

The reported result was that the camera roll and pitch were maintained around 0° even while the robot-arm tip angle changed. :contentReference[oaicite:13]{index=13}

The camera PID gains used in that experiment were:

```text
K11 = 170.0
K12 = 2.0
K13 = 0.1

K14 = 170.0
K15 = 2.0
K16 = 0.1
```

---

# 44. Reference Paper vs Simulation

The following table clearly separates the physical research system from the current simulation.

| Component | Reference Paper | Current Project |
|---|---|---|
| Quadrotor | Physical tethered quadrotor | Simplified mathematical/Simulink model |
| Roll control | PD | PD |
| Pitch control | PD | PD |
| Yaw control | PD described in paper | U4 fixed to zero in Simulink |
| Camera stabilizer | Physical 2-DOF mechanism | Simulated 2-DOF model |
| Camera controller | PID | PID |
| Camera hardware | Physical RC servos | Simplified actuator model |
| Camera sensor | Physical IMU | Not physically modeled |
| Tether sensing | Physical inclination sensing device | Mathematical tether-position function |
| Position control | Position controller described in paper | Full closed-loop position control not implemented |
| Tether dynamics | Physical tether | Geometric position relationship only |
| Quadrotor dynamics | Physical system | Simplified rotational dynamics |
| Experiment | Physical experiments | MATLAB/Simulink simulation |

This distinction is important because the project is a simulation implementation of the main control concepts rather than a complete reproduction of the physical hardware.

---

# 45. Important Implementation Differences

There are several differences between the MATLAB and Simulink implementations that should be understood before comparing their outputs.

### 45.1 Camera Actuator

`main.m` uses:

$$\ddot{\phi}_{servo}=100U_{\phi_C}-10\dot{\phi}_{servo}$$

and:

$$\ddot{\theta}_{servo}=100U_{\theta_C}-10\dot{\theta}_{servo}$$

The Simulink model uses:

$$\ddot{\phi}_{servo}=100U_{\phi_C}$$

and:

$$\ddot{\theta}_{servo}=100U_{\theta_C}$$

Therefore, the MATLAB and Simulink camera responses are based on different actuator assumptions.

### 45.2 Yaw

The parameter file contains:

```text
K5
K6
```

but the current Simulink controller does not implement the yaw PD control. The top-level Simulink model supplies:

```text
U4 = 0
```

### 45.3 Position Control

The tether position is calculated, but the current project does not feed the calculated position into a complete closed-loop position controller.

### 45.4 Simplified Dynamics

The rotational model does not include complete nonlinear quadrotor dynamics, rotor thrust equations, aerodynamic effects, or gyroscopic coupling.

---

# 46. Limitations

The current project has the following limitations:

### 46.1 Simplified Quadrotor Dynamics

The model uses independent rotational equations based on the principal moments of inertia.

It does not model:

- Full nonlinear rigid-body dynamics
- Individual motor dynamics
- Propeller aerodynamics
- Rotor thrust generation
- Gyroscopic effects
- Aerodynamic drag
- Full translational dynamics

### 46.2 Simplified Camera Actuator

The camera servo is represented using a simplified mathematical model rather than an experimentally identified physical servo model.

### 46.3 No Physical IMU Model

The reference system uses an IMU to measure camera posture, but the current simulation does not model IMU noise, bias, drift, or sensor filtering.

### 46.4 No Complete Position-Control Loop

The tether-position function calculates position, but its output is not currently used to drive a closed-loop autonomous position controller.

### 46.5 No Flexible Tether Model

The simulation treats the tether geometrically. It does not model:

- Tether mass
- Tether elasticity
- Tether tension
- Tether vibration
- Aerodynamic effects on the tether

### 46.6 Yaw Control Not Active

Yaw gains are stored in `params.m`, but yaw control is not active in the current Simulink implementation.

---

# 47. Future Work

The project can be extended in several directions.

### 47.1 Complete Quadrotor Dynamics

Implement the complete nonlinear translational and rotational equations of a quadrotor.

### 47.2 Motor and Propeller Modeling

Add:

- Motor dynamics
- Rotor thrust
- Torque generation
- Motor saturation

### 47.3 Complete Position Control

Use the tether-estimated position:

```text
x, y, z
```

as feedback for an autonomous position controller.

### 47.4 Yaw Control

Implement the yaw PD controller:

$$U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}$$

### 47.5 Realistic Camera Servo Model

Identify or obtain servo dynamics and replace the simplified actuator model with a physical servo model.

### 47.6 Sensor Modeling

Add:

- IMU noise
- Sensor bias
- Measurement delay
- Filtering

### 47.7 Tether Dynamics

Develop a dynamic tether model including tension, elasticity, and vibration.

### 47.8 Infrastructure Inspection Simulation

Create simulated bridge or tunnel environments and test the complete system during inspection trajectories.

### 47.9 Hardware-in-the-Loop Testing

Connect the Simulink controller to a hardware or flight-control platform for further validation.

The reference paper itself identifies future development involving yaw control, position control, and establishing an autonomous control system as a complete system. :contentReference[oaicite:14]{index=14}

---

# 48. Project File Structure

The repository should contain:

```text
.
├── README.md
│
├── assets/
│   └── amrita_logo.jpg
│
├── main.m
├── params.m
├── tether_pos_func.m
├── build_simulink_model.m
│
├── tethered_quadrotor_model.slx
└── tethered_quadrotor_model.slxc
```

---

# 49. File Description

| File | Description |
|---|---|
| `README.md` | Project documentation |
| `main.m` | MATLAB numerical simulation |
| `params.m` | Project parameters and controller gains |
| `tether_pos_func.m` | Tether-based position calculation |
| `build_simulink_model.m` | Programmatically creates the Simulink model |
| `tethered_quadrotor_model.slx` | Main Simulink model |
| `tethered_quadrotor_model.slxc` | Simulink generated cache file |

---

# 50. Installation Requirements

The project requires:

- MATLAB
- Simulink

The project uses standard MATLAB and Simulink functionality.

No external Python package is required for the main simulation.

---

# 51. How to Run the MATLAB Simulation

## Step 1 — Open MATLAB

Start MATLAB.

## Step 2 — Navigate to the Project Folder

Set the MATLAB current folder to the directory containing:

```text
main.m
params.m
tether_pos_func.m
build_simulink_model.m
tethered_quadrotor_model.slx
```

## Step 3 — Run the MATLAB Simulation

Execute:

```matlab
main
```

The script will:

- calculate tether position,
- simulate roll P control,
- simulate roll PD control,
- simulate pitch P control,
- simulate pitch PD control,
- simulate camera roll stabilization,
- simulate camera pitch stabilization,
- generate the plots.

---

# 52. How to Run the Simulink Model

Open the model using:

```matlab
open_system('tethered_quadrotor_model')
```

Then run the simulation from Simulink.

The model is configured with:

```text
Solver   = ode45
Start    = 0 s
Stop     = 28 s
```

---

# 53. Rebuilding the Simulink Model

If the model needs to be recreated programmatically, run:

```matlab
build_simulink_model
```

The script creates:

```text
tethered_quadrotor_model.slx
```

and automatically creates the major subsystems and their connections.

---

# 54. Quick Start

For a quick test:

```matlab
clear;
clc;
close all;

main
```

To open the Simulink model:

```matlab
open_system('tethered_quadrotor_model')
```

To rebuild the Simulink model:

```matlab
build_simulink_model
```

---

# 55. Results Interpretation

The main purpose of the simulation results is to observe controller behavior.

For the quadrotor:

```text
P Control
    vs
PD Control
```

can be compared under the same disturbances.

The derivative term provides additional feedback based on angular velocity.

For the camera:

```text
Manipulator Motion
        ↓
Camera PID
        ↓
Servo Compensation
        ↓
Net Camera Angle
```

The desired result is that the net camera roll and pitch remain close to zero while the base/manipulator angle changes.

For the tether:

```text
Tether Length
      +
Alpha
      +
Beta
      ↓
x, y, z
```

provides the estimated quadrotor position.

---

# 56. Project Contributions

This project provides a MATLAB/Simulink simulation framework for studying:

- Tethered quadrotor attitude control
- Roll control
- Pitch control
- PD controller implementation
- P vs PD comparison
- Camera roll stabilization
- Camera pitch stabilization
- PID control
- 2-DOF camera stabilization
- Tether-based position calculation
- MATLAB numerical simulation
- Simulink block-based simulation

---

# 57. Relation to Infrastructure Inspection

The motivation of the project comes from infrastructure inspection applications.

Potential inspection environments include:

```text
Bridge
  |
  +---- Underside inspection
  |
  +---- Beam inspection
  |
  +---- Structural surface inspection

Tunnel
  |
  +---- Tunnel wall inspection
  |
  +---- Ceiling inspection
  |
  +---- Restricted-access inspection
```

A stabilized camera is important because inspection requires the camera field of view to remain sufficiently stable even when the quadrotor changes attitude.

The tether can additionally provide position information in environments where GPS availability is limited.

The reference paper specifically discusses tunnels and bridges as target infrastructure-inspection environments. :contentReference[oaicite:15]{index=15}

---

# 58. Key Equations

## Quadrotor Roll

$$U_2=-K_1(\phi-\phi_d)-K_2\dot{\phi}$$

## Quadrotor Pitch

$$U_3=-K_3(\theta-\theta_d)-K_4\dot{\theta}$$

## Quadrotor Yaw

$$U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}$$

## Roll Dynamics

$$\ddot{\phi}=\frac{U_2}{I_x}$$

## Pitch Dynamics

$$\ddot{\theta}=\frac{U_3}{I_y}$$

## Yaw Dynamics

$$\ddot{\psi}=\frac{U_4}{I_z}$$

## Camera Roll PID

$$U_{\phi_C}=K_{11}e_{\phi_C}+K_{12}\int e_{\phi_C}dt+K_{13}\dot{e}_{\phi_C}$$

## Camera Pitch PID

$$U_{\theta_C}=K_{14}e_{\theta_C}+K_{15}\int e_{\theta_C}dt+K_{16}\dot{e}_{\theta_C}$$

## Camera Roll Error

$$e_{\phi_C}=\phi_{Cd}-\phi_C$$

## Camera Pitch Error

$$e_{\theta_C}=\theta_{Cd}-\theta_C$$

## Tether Position

$$x=z\tan\alpha$$

$$y=z\tan\beta$$

$$z=-\sqrt{\frac{l^2(\cos\alpha)^2(\cos\beta)^2}{(\cos\alpha)^2+(\cos\beta)^2-(\cos\alpha)^2(\cos\beta)^2}}$$

---

# 59. Technologies Used

| Technology | Purpose |
|---|---|
| MATLAB | Numerical simulation |
| Simulink | Block-based control simulation |
| MATLAB Function Block | Tether-position calculation |
| ODE45 | Simulink numerical solver |
| 3D CAD / Physical Reference | Used in the original research for inertia calculation |

---

# 60. References

1. K. Watanabe, N. Moritoki, and I. Nagai, **"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection."**

2. Y. Ouchi, K. Watanabe, K. Kinoshita, and I. Nagai, **"Position Control of an X4-Flyer Using a Tether,"** International Journal of Smart Material and Mechatronics, Vol. 1, No. 1, pp. 20–24, 2014.

3. S. Lupashin and R. D'Andrea, **"Stabilization of a Flying Vehicle on a Taut Tether using Inertial Sensing,"** IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 2432–2438, 2003.

4. S. Bouabdallah, P. Murrieri, and R. Siegwart, **"Towards Autonomous Indoor Micro VTOL,"** Autonomous Robots, Vol. 18, No. 2, pp. 171–183, 2005.

The reference paper lists these works in its bibliography. :contentReference[oaicite:16]{index=16}

---

# 61. Acknowledgement

This project is developed for academic study and is based on the concepts presented in the research paper:

**"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection."**

The original research investigates a physical tethered quadrotor equipped with a camera and demonstrates quadrotor attitude control and camera stabilization through experiments.

This repository provides a MATLAB/Simulink simulation implementation of selected control concepts from that work.

---

# 62. License

This repository is intended for academic and educational use.

The original research paper, figures, experimental data, and hardware designs remain the property of their respective authors and publisher.

This repository contains an independent MATLAB/Simulink implementation for academic study.

---

# 63. Keywords

```text
Tethered Quadrotor
Quadrotor
Drone
UAV
Attitude Control
PD Control
PID Control
Camera Stabilization
2-DOF Stabilizer
Roll Control
Pitch Control
Yaw Control
Tether Position
Tether Position Sensing
Infrastructure Inspection
MATLAB
Simulink
Quadrotor Simulation
Camera Mounted Quadrotor
Control Systems
```

---

# 64. Project Summary

```text
                    CAMERA-MOUNTED
                  TETHERED QUADROTOR
                          |
          +---------------+---------------+
          |               |               |
          v               v               v
       ATTITUDE         CAMERA          TETHER
       CONTROL        STABILIZATION     POSITION
          |               |               |
          v               v               v
       Roll/Pitch      Roll/Pitch        x,y,z
          |               |               |
          v               v               |
          PD              PID              |
          |               |               |
          +---------------+---------------+
                          |
                          v
                  MATLAB / SIMULINK
                          |
                          v
                       RESULTS
```

The project demonstrates how attitude control, camera stabilization, and tether-based position calculation can be combined into a simulation framework for a camera-mounted tethered quadrotor intended for infrastructure inspection.
