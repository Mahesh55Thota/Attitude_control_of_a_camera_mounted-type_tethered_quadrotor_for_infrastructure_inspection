<p align="center">
  <img src="assets/amrita_logo.jpg"
       alt="Amrita Vishwa Vidyapeetham Logo"
       width="300">
</p>

<h1 align="center">Attitude Control of a Camera-Mounted Tethered Quadrotor</h1>

<p align="center">
  <b>2-DOF Camera Stabilization and Tether-Based Position Sensing</b><br>
  Amrita Vishwa Vidyapeetham
</p>

---

# Team Members

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | Your Name | Your Roll Number | — |
| 2 | Team Member 2 | Roll Number | — |
| 3 | Team Member 3 | Roll Number | — |
| 4 | Team Member 4 | Roll Number | — |

---

# Abstract

This project implements a control-oriented MATLAB/Simulink simulation of a camera-mounted tethered quadrotor for infrastructure inspection. The system is based on the reference paper by Keigo Watanabe, Nao Moritoki, and Isaku Nagai, which investigates a tethered quadrotor with attitude control and a 2-DOF camera stabilizer.

The simulation includes tether-based position sensing, roll and pitch attitude control using P and PD controllers, and independent roll and pitch stabilization of the camera using PID control. The quadrotor attitude model uses the measured moments of inertia reported in the paper, while the camera stabilizer uses the reported PID gains.

The project generates simulation plots corresponding to the main control experiments in the paper and also provides an automatically generated Simulink model containing quadrotor attitude dynamics, attitude control, camera stabilization, and tether-position sensing subsystems.

---

# 1. Introduction

Quadrotors are useful for infrastructure inspection because they can perform vertical take-off, hovering, and maneuvering in areas where conventional inspection equipment may be difficult to install.

The reference work considers a tethered quadrotor for inspection of structures such as tunnels and bridges. A tether provides a method of determining the quadrotor position without relying on GPS, which can be unavailable in tunnels or beneath bridge structures.

When a camera is directly mounted on the quadrotor, changes in the body attitude can also change the camera field of view. To overcome this problem, the project uses a 2-DOF camera stabilizer that controls camera roll and pitch independently.

The complete simulation therefore focuses on three main parts:

- Tether-based position sensing
- Quadrotor attitude control
- 2-DOF camera attitude stabilization

---

# 2. Problem Statement

The objective of this project is to develop and simulate a tethered quadrotor control system capable of maintaining stable quadrotor attitude and a stable camera view during infrastructure inspection.

The system should:

- Estimate the quadrotor position from tether inclination.
- Control quadrotor roll, pitch, and yaw attitude.
- Compare P and PD attitude control.
- Stabilize camera roll and pitch using a PID controller.
- Maintain the camera attitude close to the desired reference.
- Provide MATLAB numerical simulation results.
- Provide an equivalent MATLAB/Simulink control model.

---

# 3. Objectives

The main objectives of the project are:

### 3.1 Tether-Based Position Sensing

Calculate the quadrotor position from tether length and tether inclination angles.

### 3.2 Quadrotor Attitude Control

Implement attitude control for roll, pitch, and yaw using the control equations described in the reference paper.

### 3.3 P vs PD Controller Comparison

Compare proportional control with proportional-derivative control for roll and pitch attitude stabilization.

### 3.4 Camera Stabilization

Develop a 2-DOF camera stabilizer for independent roll and pitch control.

### 3.5 PID Control

Implement PID control for the camera stabilizer to maintain the camera near the desired attitude.

### 3.6 MATLAB Simulation

Implement the mathematical models using numerical integration and generate the required plots.

### 3.7 Simulink Model

Automatically construct an interactive Simulink model containing the major subsystems of the proposed control architecture.

---

# 4. Base Paper

## Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection

The project is based on the study:

**Keigo Watanabe, Nao Moritoki, and Isaku Nagai, "Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection."**

The paper develops and experimentally evaluates a tethered quadrotor for infrastructure inspection. It specifically describes quadrotor attitude control and a 2-DOF camera stabilizer.

### Main Contributions of the Paper

- Tether-based position detection
- Quadrotor attitude control using PD control
- 2-DOF camera stabilizer
- Camera roll and pitch stabilization
- Experimental comparison of P and PD attitude control
- Experimental verification of camera stabilization

### Relation to the Present Project

The present project implements the mathematical control equations and reported physical parameters from the paper in MATLAB and Simulink.

The project is a simulation-based implementation of the control concepts and is not claimed to reproduce every physical detail of the experimental hardware.

---

# 5. Project Scope

The project focuses on the attitude-control and camera-stabilization portions of the tethered quadrotor system.

The overall control architecture is:

```text
              Tether Inclination
                     │
                     ▼
          Tether Position Sensing
                     │
                 x, y, z
                     │
                     ▼
          Quadrotor Attitude Control
             │       │       │
             ▼       ▼       ▼
           Roll    Pitch     Yaw
             │       │       │
             └───────┼───────┘
                     │
                     ▼
              Camera Mounted
                     │
              Body Disturbance
                     │
                     ▼
           2-DOF Camera PID
              │          │
              ▼          ▼
          Camera Roll  Camera Pitch
```

The implementation primarily evaluates attitude control, tether position equations, and camera stabilization.

---

# 6. Methodology

## 6.1 Simulation Environment

The project is implemented using:

- MATLAB
- MATLAB Simulink
- Numerical ODE integration
- MATLAB plotting functions

The MATLAB script performs the numerical simulations directly, while `build_simulink_model.m` creates the corresponding Simulink model.

---

## 6.2 Quadrotor Model

The quadrotor has four rotors and is represented using its principal moments of inertia.

The measured moments of inertia used in the project are:

| Parameter | Value | Unit |
|---|---:|---|
| Mass | 1.2 | kg |
| Gravity | 9.81 | m/s² |
| Tether length | 0.25 | m |
| Ix | 0.01910 | kg·m² |
| Iy | 0.01910 | kg·m² |
| Iz | 0.03083 | kg·m² |

The inertia values correspond to the measured values reported in Table I of the reference paper.

---

## 6.3 Tether Position Sensing

The tether inclination angles are represented by:

- `alpha` — inclination in the Ex direction
- `beta` — inclination in the Ey direction
- `l` — tether length

The position equations are:

$$
x = z\tan(\alpha)
$$

$$
y = z\tan(\beta)
$$

The vertical position is calculated using:

$$
z =
-\sqrt{
\frac{
l^2(\cos\alpha)^2(\cos\beta)^2
}{
(\cos\alpha)^2+(\cos\beta)^2-(\cos\alpha)^2(\cos\beta)^2
}}
$$

The implementation is contained in:

```text
tether_pos_func.m
```

For the verification case used in `main.m`:

```text
Tether length = 0.25 m
alpha = 15°
beta  = 10°
```

the calculated values are approximately:

```text
z = -0.2381 m
x = -0.0638 m
y = -0.0420 m
```

---

## 6.4 Quadrotor Attitude Control

The quadrotor attitude controller uses P/PD control.

### Roll Control

The control input is:

$$
U_2=-K_1(\phi-\phi_d)-K_2\dot{\phi}
$$

where:

- $\phi$ = roll angle
- $\phi_d$ = desired roll angle
- $K_1$ = proportional gain
- $K_2$ = derivative gain

### Pitch Control

The control input is:

$$
U_3=-K_3(\theta-\theta_d)-K_4\dot{\theta}
$$

where:

- $\theta$ = pitch angle
- $\theta_d$ = desired pitch angle
- $K_3$ = proportional gain
- $K_4$ = derivative gain

### Yaw Control

The project also defines the yaw controller using:

$$
U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}
$$

The main numerical comparison focuses on roll and pitch.

---

## 6.5 Controller Gains

The attitude-control gains used in the project are:

| Controller | Gain | Value |
|---|---|---:|
| Roll P | K1 | 0.625 |
| Roll D | K2 | 0.170 |
| Pitch P | K3 | 0.810 |
| Pitch D | K4 | 0.340 |
| Yaw P | K5 | 0.370 |
| Yaw D | K6 | 0.100 |

The roll and pitch gains are based on the gains reported in the paper's attitude-control experiment.

---

## 6.6 P and PD Control Simulation

A 10-second numerical simulation is performed for both roll and pitch.

### Roll

The simulation compares:

```text
P Controller
     vs
PD Controller
```

The roll disturbance is represented numerically using sinusoidal disturbance terms.

### Pitch

Similarly:

```text
P Controller
     vs
PD Controller
```

The pitch simulation also uses sinusoidal disturbance terms.

The purpose is to demonstrate the effect of the derivative term on attitude stabilization.

---

## 6.7 2-DOF Camera Stabilizer

The camera stabilizer consists of two independently controlled axes:

```text
             Camera Stabilizer
                  │
          ┌───────┴───────┐
          ▼               ▼
       Roll Axis       Pitch Axis
          │               │
       Servo 1         Servo 2
```

The yaw degree of freedom is not included in the stabilizer. The reference paper explains that yaw motion of the stabilizer can be replaced by the yaw control of the main quadrotor body.

This reduces:

- Mechanism weight
- Energy consumption
- Mechanical complexity

---

## 6.8 Camera PID Control

The camera roll error is:

$$
e_{\phi_C} = \phi_{Cd} - \phi_C
$$

The camera pitch error is:

$$
e_{\theta_C} = \theta_{Cd} - \theta_C
$$

The roll control input is:

$$
U_{\phi_C}
=
K_{11}e_{\phi_C}
+
K_{12}\int e_{\phi_C}\,dt
+
K_{13}\dot{e}_{\phi_C}
$$

The pitch control input is:

$$
U_{\theta_C}
=
K_{14}e_{\theta_C}
+
K_{15}\int e_{\theta_C}\,dt
+
K_{16}\dot{e}_{\theta_C}
$$

## 6.9 Camera PID Gains

The gains used in the simulation are:

| Axis | P | I | D |
|---|---:|---:|---:|
| Camera Roll | 170.0 | 2.0 | 0.1 |
| Camera Pitch | 170.0 | 2.0 | 0.1 |

These are the gain values reported for the camera stabilization experiment in the reference paper.

---

## 6.10 Camera Disturbance

The camera stabilizer is tested against a simulated manipulator/body inclination of approximately ±45°.

The disturbance profile is:

```text
0 s → 7 s       : 0° → +45°
7 s → 21 s      : +45° → -45°
21 s → 28 s     : -45° → 0°
```

The purpose is to test whether the camera can remain close to its desired zero-degree attitude while the base changes orientation.

---

## 6.11 Numerical Simulation

The `main.m` script performs the complete numerical simulation.

The process is:

```text
Load Parameters
      ↓
Tether Position Calculation
      ↓
Roll P Simulation
      ↓
Roll PD Simulation
      ↓
Pitch P Simulation
      ↓
Pitch PD Simulation
      ↓
Camera Roll PID Simulation
      ↓
Camera Pitch PID Simulation
      ↓
Generate Plots
```

The numerical integration is performed using a time-stepping approach for angular acceleration, angular velocity, and angle.

---

## 6.12 Simulink Model

The file:

```text
build_simulink_model.m
```

automatically creates the Simulink model:

```text
tethered_quadrotor_model
```

The major subsystems are:

```text
tethered_quadrotor_model
│
├── Quadrotor_Rotational_Dynamics
│
├── Quadrotor_Attitude_Controller
│
├── Camera_Stabilizer_2DOF
│
└── Tether_Position_Sensing
```

### Quadrotor Rotational Dynamics

This subsystem represents roll, pitch, and yaw angular dynamics using the principal moments of inertia.

### Quadrotor Attitude Controller

This subsystem contains the roll and pitch attitude controllers.

### Camera Stabilizer 2-DOF

This subsystem contains:

- Camera roll PID
- Camera pitch PID
- Servo dynamics

### Tether Position Sensing

This subsystem receives:

```text
alpha
beta
tether length
```

and calculates:

```text
x
y
z
```

using `tether_pos_func`.

---

## 6.13 Complete Control Loop

The complete simulation can be summarized as:

```text
        Tether Angles
             │
             ▼
     Position Calculation
             │
          x, y, z
             │
             ▼
     Quadrotor Attitude
        Controller
             │
      ┌──────┼──────┐
      ▼      ▼      ▼
    Roll   Pitch   Yaw
      │      │      │
      └──────┼──────┘
             │
             ▼
      Camera Disturbance
             │
             ▼
       2-DOF PID
        Controller
         │       │
         ▼       ▼
       Roll    Pitch
        Camera Stabilization
```

---

# 7. Experimental Setup

The numerical experiments reproduce the control conditions represented in the reference paper.

### Attitude Control

- Simulation duration: 10 seconds
- Roll: P vs PD
- Pitch: P vs PD
- Desired roll: 0°
- Desired pitch: 0°

### Camera Stabilization

- Simulation duration: 28 seconds
- Base/manipulator inclination: approximately ±45°
- Camera desired roll: 0°
- Camera desired pitch: 0°

The paper experimentally suspended the quadrotor for attitude tests and used a robot manipulator to incline the camera stabilizer by ±45° for camera stabilization tests.

---

# 8. Simulation Results

## 8.1 Roll Attitude Control

The project generates a comparison between:

```text
(a) Roll control using P controller
(b) Roll control using PD controller
```

The reference paper reports that the experimental roll error was reduced from a maximum of approximately 12.0° with P control to approximately 8.7° with PD control.

The generated MATLAB plot is used to visualize the corresponding simulated response.

---

## 8.2 Pitch Attitude Control

The project generates:

```text
(a) Pitch control using P controller
(b) Pitch control using PD controller
```

The reference paper reports a maximum pitch error of approximately -10.9° with P control and approximately 4.1° with PD control.

This demonstrates the improvement obtained by adding the derivative term.

---

## 8.3 Camera Roll Stabilization

The simulation compares:

```text
Camera roll angle
       vs
Manipulator/base angle
```

The base is moved through approximately +45° and -45°.

The objective is to maintain the camera roll close to 0° while the base angle changes.

---

## 8.4 Camera Pitch Stabilization

The same procedure is applied to camera pitch.

The simulation compares:

```text
Camera pitch angle
       vs
Manipulator/base angle
```

The PID controller attempts to compensate for the base motion and maintain the camera attitude near the desired zero-degree reference.

---

## 8.5 Reference Experimental Results

The reference paper reports that both camera roll and camera pitch were maintained at approximately 0° even when the robot-arm angle was changed.

The reported camera PID gains were:

```text
K11 = 170.0
K12 = 2.0
K13 = 0.1

K14 = 170.0
K15 = 2.0
K16 = 0.1
```

---

# 9. Performance Analysis

The project evaluates the control system using the following observations:

### Attitude Control

- P control provides proportional correction based on attitude error.
- PD control additionally uses angular velocity feedback.
- The derivative term improves damping and reduces attitude error in the reference experiment.

### Camera Stabilization

- The 2-DOF stabilizer independently controls roll and pitch.
- The PID controller compensates for base/manipulator inclination.
- The camera is intended to remain close to the desired zero-degree attitude.

### Tether Position

- Tether inclination and length are used to calculate the quadrotor position.
- The method does not require GPS position information for this calculation.

---

# 10. Key Observations

The project demonstrates the following:

1. Tether inclination can be used to estimate quadrotor position.
2. PD control provides better attitude stabilization than P control in the reference experiment.
3. A derivative term provides additional damping for roll and pitch control.
4. A 2-DOF camera stabilizer can independently control camera roll and pitch.
5. PID control can compensate for changes in the camera-supporting base.
6. Removing the stabilizer yaw axis reduces the mechanical complexity of the camera system.
7. MATLAB provides the numerical control simulation.
8. Simulink provides an interactive block-based representation of the control system.

---

# 11. Project Structure

The main project structure is:

```text
Tethered_Quadrotor_Project/
├── assets/
│   └── amrita_logo.jpg
├── README.md
├── main.m
├── params.m
├── tether_pos_func.m
├── build_simulink_model.m
└── slprj/
    └── sim/
        └── varcache/
```

### File Description

| File | Purpose |
|---|---|
| `main.m` | Main numerical simulation and plotting script |
| `params.m` | Physical parameters and controller gains |
| `tether_pos_func.m` | Tether-based position calculation |
| `build_simulink_model.m` | Automatically builds the Simulink model |
| `slprj/` | MATLAB/Simulink generated simulation files |
| `README.md` | Project documentation |

---

# 12. Technologies Used

| Technology | Purpose |
|---|---|
| MATLAB | Numerical simulation |
| Simulink | Block-based control simulation |
| MATLAB ODE/Time Integration | Attitude dynamics simulation |
| MATLAB Plotting | Simulation result visualization |
| PID Control | Camera stabilization |
| PD Control | Quadrotor attitude stabilization |

---

# 13. How to Run

## Step 1 - Open MATLAB

Open MATLAB and set the project folder as the current working directory.

## Step 2 - Run the Main Simulation

Run:

```matlab
main
```

The script will:

- Load the parameters
- Verify tether position sensing
- Simulate roll P control
- Simulate roll PD control
- Simulate pitch P control
- Simulate pitch PD control
- Simulate camera roll PID stabilization
- Simulate camera pitch PID stabilization
- Generate the result plots

## Step 3 - Build the Simulink Model

Run:

```matlab
build_simulink_model
```

This creates and opens:

```text
tethered_quadrotor_model
```

## Step 4 - Run the Simulink Simulation

After the model is opened, run the model using the Simulink **Run** button.

---

# 14. Simulation Outputs

The MATLAB simulation generates three main result figures.

### Figure 1 - Roll Attitude Control

```text
Roll P Control
Roll PD Control
```

### Figure 2 - Pitch Attitude Control

```text
Pitch P Control
Pitch PD Control
```

### Figure 3 - Camera Attitude Stabilization

```text
Camera Roll + Manipulator Angle
Camera Pitch + Manipulator Angle
```

The plots show the response of the control system over time.

---

# 15. Parameters and Controller Gains

### Physical Parameters

```text
Mass       = 1.2 kg
Gravity    = 9.81 m/s²
Tether     = 0.25 m

Ix         = 0.01910 kg·m²
Iy         = 0.01910 kg·m²
Iz         = 0.03083 kg·m²
```

### Quadrotor Attitude Controller

```text
Roll:
K1 = 0.625
K2 = 0.170

Pitch:
K3 = 0.810
K4 = 0.340

Yaw:
K5 = 0.370
K6 = 0.100
```

### Camera PID Controller

```text
Camera Roll:
K11 = 170.0
K12 = 2.0
K13 = 0.1

Camera Pitch:
K14 = 170.0
K15 = 2.0
K16 = 0.1
```

---

# 16. Limitations

The current project has the following limitations:

- The implementation is a control-oriented simulation rather than a complete physical replica of the experimental quadrotor.
- The numerical disturbance signals used for roll and pitch are simulation assumptions and are not identical to the physical disturbances in the paper.
- Full aerodynamic effects are not modeled.
- Individual rotor motor dynamics are simplified.
- Complete tether tension dynamics are not modeled.
- Full 6-DOF translational and rotational dynamics are not included in the current numerical model.
- Real IMU sensor noise is not included.
- Real RC servo dynamics are simplified.
- The current project focuses mainly on attitude, tether position sensing, and camera stabilization.

---

# 17. Future Work

Future improvements can include:

- Developing a complete 6-DOF quadrotor dynamic model.
- Modeling individual rotor thrust and motor dynamics.
- Adding realistic tether tension and tether dynamics.
- Adding IMU sensor noise and measurement filtering.
- Implementing complete x-y-z position control.
- Implementing autonomous tether-based position control.
- Adding complete yaw control and trajectory control.
- Modeling the camera and servo mechanical dynamics more accurately.
- Validating the simulation against experimental measurements.
- Implementing the complete system in a physics simulator such as PyBullet.
- Testing the controller on real quadrotor hardware.
- Integrating infrastructure-inspection camera processing.

---

# 18. Conclusion

This project implements a MATLAB/Simulink control-oriented model of a camera-mounted tethered quadrotor for infrastructure inspection.

The system combines tether-based position sensing, quadrotor roll/pitch attitude control, P and PD controller comparison, and a 2-DOF PID camera stabilizer.

The MATLAB implementation provides numerical simulations and plots, while the Simulink implementation provides a block-based representation of the major control subsystems.

The reference paper experimentally demonstrated improved roll and pitch attitude control using PD control compared with P control, and demonstrated that the 2-DOF camera stabilizer could maintain camera roll and pitch close to 0° under ±45° base inclination.

Overall, the project provides a simulation framework for studying attitude stabilization and camera stabilization of a tethered quadrotor intended for infrastructure inspection.

---

# 19. References

1. Keigo Watanabe, Nao Moritoki, and Isaku Nagai, **"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection,"** IEEE conference paper.

2. Watanabe et al., **"Development of a Camera-mounted Tethered Quadrotor for Inspecting Infrastructures,"** IECON 2016.

3. Ouchi et al., **"Position Control of an X4-Flyer Using a Tether,"** International Journal of Smart Material and Mechatronics, 2014.

4. S. Lupashin and R. D'Andrea, **"Stabilization of a Flying Vehicle on a Taut Tether using Inertial Sensing,"** IROS, 2003.

5. S. Bouabdallah, P. Murrieri, and R. Siegwart, **"Towards Autonomous Indoor Micro VTOL,"** Autonomous Robots, 2005.
