% PARAMS.M
% Parameters for Tethered Quadrotor and 2-DOF Camera Stabilizer Simulation
% Based on IEEE 2017 Paper by Keigo Watanabe, Nao Moritoki, and Isaku Nagai:
% "Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection"

%% System Parameters
quad_params.m = 1.2;              % Quadrotor mass [kg]
quad_params.g = 9.81;             % Acceleration due to gravity [m/s^2]
quad_params.l_tether = 0.25;      % Tether length l [m]

% Principal Moments of Inertia (Measured values from Table I in Paper)
quad_params.Ix = 0.01910;         % [kg*m^2]
quad_params.Iy = 0.01910;         % [kg*m^2]
quad_params.Iz = 0.03083;         % [kg*m^2]

%% Controller Gains for Quadrotor Attitude (Section II-D, Section IV)
% Roll Controller Gains (U2 = -K1*(phi - phi_d) - K2*phi_dot)
gains.K1_P_only = 0.625;          % P-only gain for Roll
gains.K1 = 0.625;                 % Roll P gain (PD control)
gains.K2 = 0.170;                 % Roll D gain (PD control)

% Pitch Controller Gains (U3 = -K3*(theta - theta_d) - K4*theta_dot)
gains.K3_P_only = 0.810;          % P-only gain for Pitch
gains.K3 = 0.810;                 % Pitch P gain (PD control)
gains.K4 = 0.340;                 % Pitch D gain (PD control)

% Yaw Controller Gains (U4 = -K5*(psi - psi_d) - K6*psi_dot)
gains.K5 = 0.370;                 % Yaw P gain
gains.K6 = 0.100;                 % Yaw D gain

%% Controller Gains for Quadrotor Outer Loop Position (Section II-D, Eqs. 7-8)
% theta_d = -K7*(x - x_d) - K8*x_dot
% phi_d   = -K9*(y - y_d) - K10*y_dot
gains.K7  = 0.50;                 % Position X proportional gain
gains.K8  = 0.20;                 % Position X derivative gain
gains.K9  = 0.50;                 % Position Y proportional gain
gains.K10 = 0.20;                 % Position Y derivative gain

%% Controller Gains for 2-DOF Camera Stabilizer (Section III-C & Section V-B)
% U_phiC   = K11*e_phiC   + K12*int(e_phiC)   + K13*e_phiC_dot
% U_thetaC = K14*e_thetaC + K15*int(e_thetaC) + K16*e_thetaC_dot
gains.K11 = 170.0;                % Camera Roll P gain
gains.K12 = 2.0;                  % Camera Roll I gain
gains.K13 = 0.1;                  % Camera Roll D gain

gains.K14 = 170.0;                % Camera Pitch P gain
gains.K15 = 2.0;                  % Camera Pitch I gain
gains.K16 = 0.1;                  % Camera Pitch D gain

%% Reference Setpoints & Disturbance Settings
refs.phi_d   = 0.0;               % Desired Roll angle [rad]
refs.theta_d = 0.0;               % Desired Pitch angle [rad]
refs.psi_d   = 0.0;               % Desired Yaw angle [rad]
refs.x_d     = 0.0;               % Desired X position [m]
refs.y_d     = 0.0;               % Desired Y position [m]

fprintf('Parameters loaded successfully.\n');
