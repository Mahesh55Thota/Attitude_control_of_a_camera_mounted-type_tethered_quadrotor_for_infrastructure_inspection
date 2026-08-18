<p align="center">
  <img src="assets/amrita_logo.jpg"
       alt="Amrita Vishwa Vidyapeetham Logo"
       width="300">
</p>

<h1 align="center">Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection</h1>

<p align="center">
  <b>MATLAB & Simulink Control Simulation</b><br>
  Amrita Vishwa Vidyapeetham
</p>

---

# Abstract

This project implements a MATLAB and Simulink simulation of a camera-mounted tethered quadrotor for infrastructure inspection. The system is based on the IEEE paper by Keigo Watanabe, Nao Moritoki, and Isaku Nagai.

The simulation covers three main parts: tether-based position detection, quadrotor attitude control using P/PD control, and a 2-DOF camera stabilizer using PID control. The camera stabilizer compensates for roll and pitch motion of the quadrotor so that the camera attitude remains close to the desired orientation.

---

# 1. Introduction

Tethered quadrotors can be used for infrastructure inspection in locations such as tunnels and bridge structures where GPS may be unavailable. The tether provides position information through its inclination angles, while the quadrotor controls its attitude by changing rotor thrust.

When a camera is mounted directly on the quadrotor, body attitude changes can disturb the camera field of view. This project therefore simulates a 2-DOF camera stabilizer that controls camera roll and pitch independently.

The model follows the control equations and parameters reported in the reference paper.

---

# 2. Problem Statement

The objective is to simulate a tethered quadrotor that can:

- Estimate its position from tether inclination and length.
- Control roll, pitch, and yaw attitude.
- Compare P and PD attitude control.
- Stabilize a mounted camera in roll and pitch.
- Maintain the camera close to a zero-degree desired attitude during platform motion.

---

# 3. Objectives

### 3.1 Tether-Based Position Detection

Calculate the quadrotor position `(x, y, z)` from tether inclination angles and tether length.

### 3.2 Quadrotor Attitude Control

Implement P and PD controllers for roll and pitch attitude control.

### 3.3 Camera Stabilization

Implement a 2-DOF PID controller for camera roll and pitch.

### 3.4 Simulation and Visualization

Generate MATLAB plots corresponding to the roll, pitch, and camera stabilization experiments described in the paper.

### 3.5 Simulink Implementation

Build an interactive Simulink model containing the quadrotor attitude dynamics, controllers, tether position sensing, and camera stabilizer.

---

# 4. Base Paper

## Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection

**Authors:** Keigo Watanabe, Nao Moritoki, Isaku Nagai  
**Affiliation:** Graduate School of Natural Science and Technology, Okayama University

The paper presents a tethered quadrotor for infrastructure inspection and describes the control of a 2-DOF camera stabilizer. The quadrotor attitude is controlled using PD control, while the camera stabilizer uses PID control.

The paper also experimentally compares P and PD attitude control and demonstrates that the camera roll and pitch can be maintained near 0° while the supporting mechanism is inclined by ±45°.

---

# 5. System Overview

```text
                 TETHERED QUADROTOR SYSTEM
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          ▼                ▼                ▼
   Tether Position    Quadrotor Attitude   Camera
      Detection           Control         Stabilizer
          │                │                │
      α, β, l          Roll/Pitch/Yaw     Roll/Pitch
          │                │                │
          ▼                ▼                ▼
       x, y, z           P / PD            PID
          │                │                │
          └────────────────┼────────────────┘
                           ▼
                    MATLAB / Simulink
```

---

# 6. Methodology

## 6.1 Tether Position Detection

The tether inclination angles are represented by `α` and `β`, and the tether length is `l`.

The position is calculated using:

$$
x=z\tan\alpha
$$

$$
y=z\tan\beta
$$

and:

$$
z=-\sqrt{
\frac{l^2(\cos\alpha)^2(\cos\beta)^2}
{(\cos\alpha)^2+(\cos\beta)^2-(\cos\alpha)^2(\cos\beta)^2}
}
$$

The MATLAB helper function `tether_pos_func.m` implements these equations.

---

## 6.2 Quadrotor Attitude Control

The quadrotor attitude controller uses:

- Roll angle `φ`
- Pitch angle `θ`
- Yaw angle `ψ`

The roll controller is:

$$
U_2=-K_1(\phi-\phi_d)-K_2\dot{\phi}
$$

The pitch controller is:

$$
U_3=-K_3(\theta-\theta_d)-K_4\dot{\theta}
$$

The yaw controller is:

$$
U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}
$$

The project compares P-only and PD control for roll and pitch.

---

## 6.3 Position Control

The paper defines the desired pitch and roll angles from position error as:

$$
\theta_d=-K_7(x-x_d)-K_8\dot{x}
$$

$$
\phi_d=-K_9(y-y_d)-K_{10}\dot{y}
$$

These equations are included in the project parameter definitions for the outer position-control loop.

---

## 6.4 2-DOF Camera Stabilizer

The camera stabilizer has two controlled degrees of freedom:

```text
Camera Stabilizer
       │
       ├── Roll  (φC)
       │
       └── Pitch (θC)
```

The camera roll error is:

$$
e_{\phi C}=\phi_{Cd}-\phi_C
$$

and the pitch error is:

$$
e_{\theta C}=\theta_{Cd}-\theta_C
$$

The PID controllers are:

$$
U_{\phi C}
=
K_{11}e_{\phi C}
+
K_{12}\int e_{\phi C}dt
+
K_{13}\dot e_{\phi C}
$$

$$
U_{\theta C}
=
K_{14}e_{\theta C}
+
K_{15}\int e_{\theta C}dt
+
K_{16}\dot e_{\theta C}
$$

The stabilizer therefore compensates for platform inclination while maintaining the desired camera orientation.

---

# 7. Simulation Parameters

| Parameter | Value |
| :--- | :---: |
| Quadrotor mass | 1.2 kg |
| Gravity | 9.81 m/s² |
| Tether length | 0.25 m |
| Measured `Ix` | 0.01910 kg·m² |
| Measured `Iy` | 0.01910 kg·m² |
| Measured `Iz` | 0.03083 kg·m² |
| Desired roll | 0° |
| Desired pitch | 0° |
| Desired yaw | 0° |

### Attitude Controller Gains

| Gain | Value |
| :--- | :---: |
| `K1` Roll P | 0.625 |
| `K2` Roll D | 0.170 |
| `K3` Pitch P | 0.810 |
| `K4` Pitch D | 0.340 |
| `K5` Yaw P | 0.370 |
| `K6` Yaw D | 0.100 |

### Position Controller Gains

| Gain | Value |
| :--- | :---: |
| `K7` | 0.50 |
| `K8` | 0.20 |
| `K9` | 0.50 |
| `K10` | 0.20 |

### Camera PID Gains

| Gain | Value |
| :--- | :---: |
| `K11` Roll P | 170.0 |
| `K12` Roll I | 2.0 |
| `K13` Roll D | 0.1 |
| `K14` Pitch P | 170.0 |
| `K15` Pitch I | 2.0 |
| `K16` Pitch D | 0.1 |

---

# 8. Simulation Workflow

```text
Load Parameters
       ↓
Tether Position Calculation
       ↓
Roll P vs PD Simulation
       ↓
Pitch P vs PD Simulation
       ↓
Camera Roll PID Simulation
       ↓
Camera Pitch PID Simulation
       ↓
Generate MATLAB Plots
       ↓
Build Simulink Model
```

---

# 9. Simulation Outputs

The MATLAB simulation generates three main result figures.

### Figure 8 — Roll Attitude Control

```text
P Control
   vs
PD Control
```

This compares the roll response obtained using P and PD control.

### Figure 9 — Pitch Attitude Control

```text
P Control
   vs
PD Control
```

This compares the pitch response obtained using P and PD control.

### Figure 11 — Camera Attitude

```text
Camera Roll      vs      Manipulator Angle
Camera Pitch     vs      Manipulator Angle
```

The camera is commanded to maintain approximately 0° while the supporting mechanism is moved through the ±45° range used in the reference experiment.

---

# 10. Simulink Model

The project includes a MATLAB script that programmatically builds the Simulink model:

```text
tethered_quadrotor_model.slx
```

The main model contains:

```text
Tether Position Sensing
          │
          ▼
Quadrotor Attitude Controller
          │
          ▼
Quadrotor Rotational Dynamics
          │
          ▼
Camera Stabilizer
   ┌──────┴──────┐
   ▼             ▼
Roll PID      Pitch PID
```

The Simulink model provides a graphical representation of the control system and allows the control blocks and signals to be inspected.

---

# 11. Project Structure

```text
Tethered_Quadrotor_Project/
├── assets/
│   └── amrita_logo.jpg
├── README.md
├── main.m
├── params.m
├── tether_pos_func.m
├── build_simulink_model.m
└── tethered_quadrotor_model.slx
```

---

# 12. Files Description

| File | Description |
| :--- | :--- |
| `main.m` | Master MATLAB script for numerical simulation and result plots |
| `params.m` | Physical parameters, controller gains, and reference values |
| `tether_pos_func.m` | Calculates tether-based `x`, `y`, and `z` position |
| `build_simulink_model.m` | Programmatically builds the Simulink model |
| `tethered_quadrotor_model.slx` | Main graphical Simulink control model |
| `README.md` | Project documentation |

---

# 13. How to Run

## Option 1 — MATLAB Numerical Simulation

Open MATLAB and navigate to the project folder.

Run:

```matlab
main
```

The script will:

- Load the system parameters.
- Verify tether-based position sensing.
- Simulate roll P and PD control.
- Simulate pitch P and PD control.
- Simulate the 2-DOF camera stabilizer.
- Generate the corresponding plots.

---

## Option 2 — Simulink Simulation

Run:

```matlab
build_simulink_model
```

This creates and opens:

```text
tethered_quadrotor_model.slx
```

Then click **Run (▶)** in Simulink to execute the graphical model.

---

# 14. Reference Experimental Results

The reference paper reported:

| Experiment | P Control | PD Control |
| :--- | :---: | :---: |
| Roll maximum error | 12.0° | 8.7° |
| Pitch maximum error | -10.9° | 4.1° |

The paper therefore reported improved attitude-error performance using PD control compared with P control.

For the camera stabilizer, the experimental results showed that camera roll and pitch could be maintained near 0° while the robot arm was inclined by ±45°.

---

# 15. Key Observations

1. PD control provides damping through the derivative term.
2. The tether inclination provides information for estimating quadrotor position.
3. The camera stabilizer uses two controlled degrees of freedom: roll and pitch.
4. PID control compensates for camera orientation changes.
5. Removing the stabilizer's yaw degree of freedom reduces the stabilizer's complexity, weight, and energy requirement.
6. MATLAB provides the numerical simulation while Simulink provides the graphical control-system implementation.

---

# 16. Limitations

- The current implementation is a control-oriented numerical simulation.
- The tether is represented through the position equations rather than a complete physical tether-force model.
- The quadrotor model does not represent complete rotor, motor, aerodynamic, and 6-DOF physical dynamics.
- The simulation uses mathematical disturbance signals for the P/PD comparison.
- The camera stabilizer is represented through simplified rotational dynamics.
- The results should not be interpreted as an exact reproduction of the physical experimental setup.

---

# 17. Future Work

- Develop a complete 6-DOF quadrotor dynamic model.
- Add individual rotor and motor dynamics.
- Include tether tension and tether-force dynamics.
- Add full x-y-z position control.
- Add autonomous yaw control.
- Include IMU and sensor-noise models.
- Implement the complete quadrotor and camera system in a physics simulator.
- Validate the controller using real hardware.

---

# 18. Technologies Used

| Technology | Purpose |
| :--- | :--- |
| MATLAB | Numerical simulation |
| Simulink | Graphical control-system simulation |
| MATLAB ODE / Numerical Integration | Attitude and camera dynamics |
| Control Systems | P, PD, and PID controllers |

---

# 19. References

1. K. Watanabe, N. Moritoki, and I. Nagai, **"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection,"** IEEE, 2017.
2. Y. Ouchi, K. Watanabe, K. Kinoshita, and I. Nagai, **"Position Control of an X4-Flyer Using a Tether,"** International Journal of Smart Material and Mechatronics, 2014.
3. S. Bouabdallah, P. Murrieri, and R. Siegwart, **"Towards Autonomous Indoor Micro VTOL,"** Autonomous Robots, 2005.
