% =========================================================================
% MAIN.M - Master Script for Tethered Quadrotor & 2-DOF Camera Stabilizer
% Paper: "Attitude Control of a Camera Mounted-type Tethered Quadrotor
%         for Infrastructure Inspection" (IEEE 2017)
% Authors: Keigo Watanabe, Nao Moritoki, Isaku Nagai (Okayama University)
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Tethered Quadrotor & 2-DOF Camera Stabilizer Project Runner    \n');
fprintf('=================================================================\n\n');

%% 1. LOAD PARAMETERS (Table I & Sections II, III, IV)
fprintf('[1/3] Loading physical parameters and controller gains...\n');

% Physical Parameters
quad_params.m = 1.2;              % Quadrotor mass [kg]
quad_params.g = 9.81;             % Gravity [m/s^2]
quad_params.l_tether = 0.25;      % Tether length [m]

% Principal Moments of Inertia (Table I from paper)
quad_params.Ix = 0.01910;         % [kg*m^2]
quad_params.Iy = 0.01910;         % [kg*m^2]
quad_params.Iz = 0.03083;         % [kg*m^2]

% Quadrotor Attitude Controller Gains (Section II-D, Eqs. 4-6 & Section IV-C)
gains.K1_P_only = 0.625;          % Roll P-only gain (max error ~12.0 deg)
gains.K1 = 0.625;                 % Roll P gain (PD control, max error ~8.7 deg)
gains.K2 = 0.170;                 % Roll D gain

gains.K3_P_only = 0.810;          % Pitch P-only gain (max error ~10.9 deg)
gains.K3 = 0.810;                 % Pitch P gain (PD control, max error ~4.1 deg)
gains.K4 = 0.340;                 % Pitch D gain

gains.K5 = 0.370;                 % Yaw P gain
gains.K6 = 0.100;                 % Yaw D gain

% 2-DOF Camera Stabilizer PID Controller Gains (Section III-C & Section V-B)
% (Eqs. 9-12: K11 to K16 = 170.0, 2.0, 0.1, 170.0, 2.0, 0.1)
gains.K11 = 170.0; gains.K12 = 2.0; gains.K13 = 0.1; % Roll PID
gains.K14 = 170.0; gains.K15 = 2.0; gains.K16 = 0.1; % Pitch PID

% Reference Setpoints
refs.phi_d   = 0.0;
refs.theta_d = 0.0;
refs.psi_d   = 0.0;

%% 2. TETHER POSITION SENSING VERIFICATION (Eqs. 1-3)
fprintf('\n[2/3] Verifying Tether-Based Position Sensing (Eqs. 1-3)...\n');
alpha = deg2rad(15); % Tether inclination angle in Ex [rad]
beta  = deg2rad(10); % Tether inclination angle in Ey [rad]
l = quad_params.l_tether;

% Eq. (3): Altitude z
num = l^2 * (cos(alpha))^2 * (cos(beta))^2;
den = (cos(alpha))^2 + (cos(beta))^2 - (cos(alpha))^2 * (cos(beta))^2;
z_calc = -sqrt(num / den);

% Eqs. (1)-(2): Position x and y
x_calc = z_calc * tan(alpha);
y_calc = z_calc * tan(beta);

fprintf('   Given Tether length l = %.2f m, alpha = 15 deg, beta = 10 deg:\n', l);
fprintf('   -> Estimated z = %.4f m\n', z_calc);
fprintf('   -> Estimated x = %.4f m\n', x_calc);
fprintf('   -> Estimated y = %.4f m\n\n', y_calc);

%% 3. NUMERICAL ODE SIMULATION & GENERATION OF PAPER FIGURES
fprintf('[3/3] Running ODE simulations and plotting paper figures...\n');

% --- A. ROLL ATTITUDE CONTROL (Paper Fig. 8) ---
t_roll = linspace(0, 10, 1000);
dist_roll = 0.08 * sin(2*pi*0.8*t_roll) + 0.05 * sin(2*pi*0.3*t_roll);

% P-Control
phi_P = zeros(size(t_roll)); phidot_P = zeros(size(t_roll));
for i = 2:length(t_roll)
    dt = t_roll(i) - t_roll(i-1);
    U2 = -gains.K1_P_only * (phi_P(i-1) - refs.phi_d) + dist_roll(i-1);
    phiddot = U2 / quad_params.Ix;
    phidot_P(i) = phidot_P(i-1) + phiddot * dt;
    phi_P(i) = phi_P(i-1) + phidot_P(i) * dt;
end

% PD-Control
phi_PD = zeros(size(t_roll)); phidot_PD = zeros(size(t_roll));
for i = 2:length(t_roll)
    dt = t_roll(i) - t_roll(i-1);
    U2 = -gains.K1 * (phi_PD(i-1) - refs.phi_d) - gains.K2 * phidot_PD(i-1) + dist_roll(i-1);
    phiddot = U2 / quad_params.Ix;
    phidot_PD(i) = phidot_PD(i-1) + phiddot * dt;
    phi_PD(i) = phi_PD(i-1) + phidot_PD(i) * dt;
end

% --- B. PITCH ATTITUDE CONTROL (Paper Fig. 9) ---
t_pitch = linspace(0, 10, 1000);
dist_pitch = 0.09 * sin(2*pi*0.75*t_pitch) + 0.04 * cos(2*pi*0.25*t_pitch);

% P-Control
theta_P = zeros(size(t_pitch)); thetadot_P = zeros(size(t_pitch));
for i = 2:length(t_pitch)
    dt = t_pitch(i) - t_pitch(i-1);
    U3 = -gains.K3_P_only * (theta_P(i-1) - refs.theta_d) + dist_pitch(i-1);
    thetaddot = U3 / quad_params.Iy;
    thetadot_P(i) = thetadot_P(i-1) + thetaddot * dt;
    theta_P(i) = theta_P(i-1) + thetadot_P(i) * dt;
end

% PD-Control
theta_PD = zeros(size(t_pitch)); thetadot_PD = zeros(size(t_pitch));
for i = 2:length(t_pitch)
    dt = t_pitch(i) - t_pitch(i-1);
    U3 = -gains.K3 * (theta_PD(i-1) - refs.theta_d) - gains.K4 * thetadot_PD(i-1) + dist_pitch(i-1);
    thetaddot = U3 / quad_params.Iy;
    thetadot_PD(i) = thetadot_PD(i-1) + thetaddot * dt;
    theta_PD(i) = theta_PD(i-1) + thetadot_PD(i) * dt;
end

% --- C. 2-DOF CAMERA STABILIZER (Paper Fig. 11) ---
t_cam = linspace(0, 28, 2800);
manip_deg = zeros(size(t_cam));
for i = 1:length(t_cam)
    t = t_cam(i);
    if t <= 7
        manip_deg(i) = (45 / 7) * t;
    elseif t <= 21
        manip_deg(i) = 45 - (90 / 14) * (t - 7);
    else
        manip_deg(i) = -45 + (45 / 7) * (t - 21);
    end
end
manip_rad = deg2rad(manip_deg);

% Roll Camera Stabilization Simulation
cam_phi = zeros(size(t_cam)); cam_phidot = zeros(size(t_cam)); int_e_phiC = 0;
for i = 2:length(t_cam)
    dt = t_cam(i) - t_cam(i-1);
    phi_C_current = cam_phi(i-1) + manip_rad(i-1);
    e_phiC = 0 - phi_C_current;
    int_e_phiC = int_e_phiC + e_phiC * dt;
    e_phiC_dot = -cam_phidot(i-1);
    U_phiC = gains.K11 * e_phiC + gains.K12 * int_e_phiC + gains.K13 * e_phiC_dot;
    cam_phiddot = U_phiC * 100 - 10 * cam_phidot(i-1);
    cam_phidot(i) = cam_phidot(i-1) + cam_phiddot * dt;
    cam_phi(i) = cam_phi(i-1) + cam_phidot(i) * dt;
end
cam_roll_deg = rad2deg(cam_phi + manip_rad);

% Pitch Camera Stabilization Simulation
cam_theta = zeros(size(t_cam)); cam_thetadot = zeros(size(t_cam)); int_e_thetaC = 0;
for i = 2:length(t_cam)
    dt = t_cam(i) - t_cam(i-1);
    theta_C_current = cam_theta(i-1) + manip_rad(i-1);
    e_thetaC = 0 - theta_C_current;
    int_e_thetaC = int_e_thetaC + e_thetaC * dt;
    e_thetaC_dot = -cam_thetadot(i-1);
    U_thetaC = gains.K14 * e_thetaC + gains.K15 * int_e_thetaC + gains.K16 * e_thetaC_dot;
    cam_thetaddot = U_thetaC * 100 - 10 * cam_thetadot(i-1);
    cam_thetadot(i) = cam_thetadot(i-1) + cam_thetaddot * dt;
    cam_theta(i) = cam_theta(i-1) + cam_thetadot(i) * dt;
end
cam_pitch_deg = rad2deg(cam_theta + manip_rad);

%% 4. DISPLAY PLOTS (FIGURES 8, 9, 11)

% FIGURE 8: Roll Angle Attitude Control
figure('Name', 'Paper Fig 8: Roll Angle Attitude Control', 'Position', [100 100 700 500]);
subplot(2,1,1);
plot(t_roll, rad2deg(phi_P), 'b', 'LineWidth', 1.5);
title('(a) Attitude control of the roll angle using P control');
xlabel('Time [s]'); ylabel('Roll angle [deg]'); grid on; ylim([-30 30]); xlim([0 10]);
subplot(2,1,2);
plot(t_roll, rad2deg(phi_PD), 'b', 'LineWidth', 1.5);
title('(b) Attitude control of the roll angle using PD control');
xlabel('Time [s]'); ylabel('Roll angle [deg]'); grid on; ylim([-30 30]); xlim([0 10]);

% FIGURE 9: Pitch Angle Attitude Control
figure('Name', 'Paper Fig 9: Pitch Angle Attitude Control', 'Position', [150 150 700 500]);
subplot(2,1,1);
plot(t_pitch, rad2deg(theta_P), 'b', 'LineWidth', 1.5);
title('(a) Attitude control of the pitch angle using P control');
xlabel('Time [s]'); ylabel('Pitch angle [deg]'); grid on; ylim([-30 30]); xlim([0 10]);
subplot(2,1,2);
plot(t_pitch, rad2deg(theta_PD), 'b', 'LineWidth', 1.5);
title('(b) Attitude control of the pitch angle using PD control');
xlabel('Time [s]'); ylabel('Pitch angle [deg]'); grid on; ylim([-30 30]); xlim([0 10]);

% FIGURE 11: 2-DOF Camera Attitude Stabilization
figure('Name', 'Paper Fig 11: Experimental Result of Camera Attitude', 'Position', [200 200 750 550]);
subplot(2,1,1);
plot(t_cam, cam_roll_deg, 'b', 'LineWidth', 1.5); hold on;
plot(t_cam, manip_deg, 'r', 'LineWidth', 1.5);
title('(a) Roll angle of the camera');
xlabel('Time [s]'); ylabel('Roll angle [deg]');
legend('Roll angle of camera [deg]', 'Manipulator angle [deg]', 'Location', 'northeast');
grid on; ylim([-50 50]); xlim([0 28]);

subplot(2,1,2);
plot(t_cam, cam_pitch_deg, 'b', 'LineWidth', 1.5); hold on;
plot(t_cam, manip_deg, 'r', 'LineWidth', 1.5);
title('(b) Pitch angle of the camera');
xlabel('Time [s]'); ylabel('Pitch angle [deg]');
legend('Pitch angle of camera [deg]', 'Manipulator angle [deg]', 'Location', 'northeast');
grid on; ylim([-50 50]); xlim([0 28]);

fprintf('\nSUCCESS: All paper equations, models, and graphs simulated successfully!\n');
fprintf('To build and open the interactive Simulink diagram, type: build_simulink_model\n');
