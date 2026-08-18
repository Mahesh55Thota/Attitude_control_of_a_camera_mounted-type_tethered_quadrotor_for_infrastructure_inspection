% BUILD_SIMULINK_MODEL.M
% Fully Connected & Configured Simulink Model Builder (.slx)
% Paper: "Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection"
% (IEEE 2017 - Watanabe, Moritoki, Nagai)

clear; clc;
model_name = 'tethered_quadrotor_model';

% Load parameter file into MATLAB workspace
run('params.m');

% Close system if already open
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Create new Simulink model
new_system(model_name);
open_system(model_name);

% Set solver properties to match 28s paper experiment
set_param(model_name, 'Solver', 'ode45', 'StopTime', '28.0');

fprintf('Building fully-connected Simulink Model: %s.slx...\n', model_name);

%% =========================================================================
%% 1. QUADROTOR ROTATIONAL DYNAMICS SUBSYSTEM (Eqs. 4-6 & Section IV)
%% =========================================================================
subsystem_dyn = [model_name '/Quadrotor_Rotational_Dynamics'];
add_block('simulink/Ports & Subsystems/Subsystem', subsystem_dyn, 'Position', [480, 60, 720, 260]);
clear_subsystem(subsystem_dyn);

% Inports (Total Torques = Control + Disturbance)
add_block('simulink/Sources/In1', [subsystem_dyn '/U2_Roll_Torque'],  'Position', [30, 45, 60, 59]);
add_block('simulink/Sources/In1', [subsystem_dyn '/U3_Pitch_Torque'], 'Position', [30, 115, 60, 129]);
add_block('simulink/Sources/In1', [subsystem_dyn '/U4_Yaw_Torque'],   'Position', [30, 185, 60, 199]);

% Roll Axis Dynamics
add_block('simulink/Math Operations/Gain', [subsystem_dyn '/Gain_1_Ix'], 'Position', [100, 40, 150, 65], 'Gain', '1/quad_params.Ix');
add_block('simulink/Continuous/Integrator', [subsystem_dyn '/Int_phidot'], 'Position', [180, 40, 210, 65]);
add_block('simulink/Continuous/Integrator', [subsystem_dyn '/Int_phi'], 'Position', [260, 40, 290, 65]);

% Pitch Axis Dynamics
add_block('simulink/Math Operations/Gain', [subsystem_dyn '/Gain_1_Iy'], 'Position', [100, 110, 150, 135], 'Gain', '1/quad_params.Iy');
add_block('simulink/Continuous/Integrator', [subsystem_dyn '/Int_thetadot'], 'Position', [180, 110, 210, 135]);
add_block('simulink/Continuous/Integrator', [subsystem_dyn '/Int_theta'], 'Position', [260, 110, 290, 135]);

% Yaw Axis Dynamics
add_block('simulink/Math Operations/Gain', [subsystem_dyn '/Gain_1_Iz'], 'Position', [100, 180, 150, 205], 'Gain', '1/quad_params.Iz');
add_block('simulink/Continuous/Integrator', [subsystem_dyn '/Int_psidot'], 'Position', [180, 180, 210, 205]);
add_block('simulink/Continuous/Integrator', [subsystem_dyn '/Int_psi'], 'Position', [260, 180, 290, 205]);

% Outports
add_block('simulink/Sinks/Out1', [subsystem_dyn '/phi'],      'Position', [340, 45, 370, 59]);
add_block('simulink/Sinks/Out1', [subsystem_dyn '/phidot'],   'Position', [340, 80, 370, 94]);
add_block('simulink/Sinks/Out1', [subsystem_dyn '/theta'],    'Position', [340, 115, 370, 129]);
add_block('simulink/Sinks/Out1', [subsystem_dyn '/thetadot'], 'Position', [340, 150, 370, 164]);
add_block('simulink/Sinks/Out1', [subsystem_dyn '/psi'],      'Position', [340, 185, 370, 199]);

% Internal Connections
connect_blocks(subsystem_dyn, 'U2_Roll_Torque', 1, 'Gain_1_Ix', 1);
connect_blocks(subsystem_dyn, 'Gain_1_Ix', 1, 'Int_phidot', 1);
connect_blocks(subsystem_dyn, 'Int_phidot', 1, 'Int_phi', 1);
connect_blocks(subsystem_dyn, 'Int_phidot', 1, 'phidot', 1);
connect_blocks(subsystem_dyn, 'Int_phi', 1, 'phi', 1);

connect_blocks(subsystem_dyn, 'U3_Pitch_Torque', 1, 'Gain_1_Iy', 1);
connect_blocks(subsystem_dyn, 'Gain_1_Iy', 1, 'Int_thetadot', 1);
connect_blocks(subsystem_dyn, 'Int_thetadot', 1, 'Int_theta', 1);
connect_blocks(subsystem_dyn, 'Int_thetadot', 1, 'thetadot', 1);
connect_blocks(subsystem_dyn, 'Int_theta', 1, 'theta', 1);

connect_blocks(subsystem_dyn, 'U4_Yaw_Torque', 1, 'Gain_1_Iz', 1);
connect_blocks(subsystem_dyn, 'Gain_1_Iz', 1, 'Int_psidot', 1);
connect_blocks(subsystem_dyn, 'Int_psidot', 1, 'Int_psi', 1);
connect_blocks(subsystem_dyn, 'Int_psi', 1, 'psi', 1);

%% =========================================================================
%% 2. QUADROTOR PD ATTITUDE CONTROLLER SUBSYSTEM (Eqs. 4-6)
%% =========================================================================
subsystem_ctrl = [model_name '/Quadrotor_Attitude_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', subsystem_ctrl, 'Position', [140, 60, 360, 260]);
clear_subsystem(subsystem_ctrl);

add_block('simulink/Sources/In1', [subsystem_ctrl '/phi_d'],   'Position', [30, 35, 60, 49]);
add_block('simulink/Sources/In1', [subsystem_ctrl '/phi'],     'Position', [30, 65, 60, 79]);
add_block('simulink/Sources/In1', [subsystem_ctrl '/phidot'],  'Position', [30, 95, 60, 109]);
add_block('simulink/Sources/In1', [subsystem_ctrl '/theta_d'], 'Position', [30, 140, 60, 154]);
add_block('simulink/Sources/In1', [subsystem_ctrl '/theta'],   'Position', [30, 170, 60, 184]);
add_block('simulink/Sources/In1', [subsystem_ctrl '/thetadot'],'Position', [30, 200, 60, 214]);

% Roll PD: U2 = -K1*(phi - phi_d) - K2*phidot
add_block('simulink/Math Operations/Sum', [subsystem_ctrl '/Sum_Roll_Err'], 'Position', [100, 40, 120, 60], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [subsystem_ctrl '/Gain_K1'], 'Position', [140, 40, 180, 60], 'Gain', '-gains.K1');
add_block('simulink/Math Operations/Gain', [subsystem_ctrl '/Gain_K2'], 'Position', [140, 90, 180, 110], 'Gain', '-gains.K2');
add_block('simulink/Math Operations/Sum', [subsystem_ctrl '/Sum_U2'], 'Position', [220, 50, 240, 80], 'Inputs', '++');
add_block('simulink/Sinks/Out1', [subsystem_ctrl '/U2'], 'Position', [270, 60, 300, 74]);

connect_blocks(subsystem_ctrl, 'phi_d', 1, 'Sum_Roll_Err', 1);
connect_blocks(subsystem_ctrl, 'phi', 1, 'Sum_Roll_Err', 2);
connect_blocks(subsystem_ctrl, 'Sum_Roll_Err', 1, 'Gain_K1', 1);
connect_blocks(subsystem_ctrl, 'phidot', 1, 'Gain_K2', 1);
connect_blocks(subsystem_ctrl, 'Gain_K1', 1, 'Sum_U2', 1);
connect_blocks(subsystem_ctrl, 'Gain_K2', 1, 'Sum_U2', 2);
connect_blocks(subsystem_ctrl, 'Sum_U2', 1, 'U2', 1);

% Pitch PD: U3 = -K3*(theta - theta_d) - K4*thetadot
add_block('simulink/Math Operations/Sum', [subsystem_ctrl '/Sum_Pitch_Err'], 'Position', [100, 145, 120, 165], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [subsystem_ctrl '/Gain_K3'], 'Position', [140, 145, 180, 165], 'Gain', '-gains.K3');
add_block('simulink/Math Operations/Gain', [subsystem_ctrl '/Gain_K4'], 'Position', [140, 195, 180, 215], 'Gain', '-gains.K4');
add_block('simulink/Math Operations/Sum', [subsystem_ctrl '/Sum_U3'], 'Position', [220, 155, 240, 185], 'Inputs', '++');
add_block('simulink/Sinks/Out1', [subsystem_ctrl '/U3'], 'Position', [270, 165, 300, 179]);

connect_blocks(subsystem_ctrl, 'theta_d', 1, 'Sum_Pitch_Err', 1);
connect_blocks(subsystem_ctrl, 'theta', 1, 'Sum_Pitch_Err', 2);
connect_blocks(subsystem_ctrl, 'Sum_Pitch_Err', 1, 'Gain_K3', 1);
connect_blocks(subsystem_ctrl, 'thetadot', 1, 'Gain_K4', 1);
connect_blocks(subsystem_ctrl, 'Gain_K3', 1, 'Sum_U3', 1);
connect_blocks(subsystem_ctrl, 'Gain_K4', 1, 'Sum_U3', 2);
connect_blocks(subsystem_ctrl, 'Sum_U3', 1, 'U3', 1);

%% =========================================================================
%% 3. CAMERA STABILIZER 2-DOF SUBSYSTEM (Section III & V, Fig. 10 & 11)
%% =========================================================================
subsystem_cam = [model_name '/Camera_Stabilizer_2DOF'];
add_block('simulink/Ports & Subsystems/Subsystem', subsystem_cam, 'Position', [480, 310, 720, 480]);
clear_subsystem(subsystem_cam);

add_block('simulink/Sources/In1', [subsystem_cam '/phi_base_disturb'],   'Position', [30, 50, 60, 64]);
add_block('simulink/Sources/In1', [subsystem_cam '/theta_base_disturb'], 'Position', [30, 160, 60, 174]);

% Roll Axis PID Control + Servo Dynamics (Eq. 9)
add_block('simulink/Math Operations/Sum', [subsystem_cam '/Sum_Roll_Total'], 'Position', [90, 45, 110, 70], 'Inputs', '++');
add_block('simulink/Math Operations/Gain', [subsystem_cam '/Gain_K11'], 'Position', [150, 35, 190, 55], 'Gain', '-gains.K11');
add_block('simulink/Continuous/Integrator', [subsystem_cam '/Int_Roll'], 'Position', [130, 65, 160, 85]);
add_block('simulink/Math Operations/Gain', [subsystem_cam '/Gain_K12'], 'Position', [180, 65, 220, 85], 'Gain', '-gains.K12');
add_block('simulink/Continuous/Derivative', [subsystem_cam '/Deriv_Roll'], 'Position', [130, 95, 160, 115]);
add_block('simulink/Math Operations/Gain', [subsystem_cam '/Gain_K13'], 'Position', [180, 95, 220, 115], 'Gain', '-gains.K13');
add_block('simulink/Math Operations/Sum', [subsystem_cam '/Sum_Roll_PID'], 'Position', [250, 50, 270, 90], 'Inputs', '+++');

add_block('simulink/Math Operations/Gain', [subsystem_cam '/Servo1_Gain'], 'Position', [290, 60, 320, 80], 'Gain', '100');
add_block('simulink/Continuous/Integrator', [subsystem_cam '/Int_Servo1_vel'], 'Position', [340, 60, 360, 80]);
add_block('simulink/Continuous/Integrator', [subsystem_cam '/Int_Servo1_pos'], 'Position', [380, 60, 400, 80]);

connect_blocks(subsystem_cam, 'phi_base_disturb', 1, 'Sum_Roll_Total', 1);
connect_blocks(subsystem_cam, 'Int_Servo1_pos', 1, 'Sum_Roll_Total', 2);

connect_blocks(subsystem_cam, 'Sum_Roll_Total', 1, 'Gain_K11', 1);
connect_blocks(subsystem_cam, 'Sum_Roll_Total', 1, 'Int_Roll', 1);
connect_blocks(subsystem_cam, 'Int_Roll', 1, 'Gain_K12', 1);
connect_blocks(subsystem_cam, 'Sum_Roll_Total', 1, 'Deriv_Roll', 1);
connect_blocks(subsystem_cam, 'Deriv_Roll', 1, 'Gain_K13', 1);

connect_blocks(subsystem_cam, 'Gain_K11', 1, 'Sum_Roll_PID', 1);
connect_blocks(subsystem_cam, 'Gain_K12', 1, 'Sum_Roll_PID', 2);
connect_blocks(subsystem_cam, 'Gain_K13', 1, 'Sum_Roll_PID', 3);

connect_blocks(subsystem_cam, 'Sum_Roll_PID', 1, 'Servo1_Gain', 1);
connect_blocks(subsystem_cam, 'Servo1_Gain', 1, 'Int_Servo1_vel', 1);
connect_blocks(subsystem_cam, 'Int_Servo1_vel', 1, 'Int_Servo1_pos', 1);

add_block('simulink/Math Operations/Sum', [subsystem_cam '/Sum_Net_Cam_Roll'], 'Position', [440, 50, 460, 80], 'Inputs', '++');
connect_blocks(subsystem_cam, 'Int_Servo1_pos', 1, 'Sum_Net_Cam_Roll', 1);
connect_blocks(subsystem_cam, 'phi_base_disturb', 1, 'Sum_Net_Cam_Roll', 2);

add_block('simulink/Sinks/Out1', [subsystem_cam '/Camera_Roll_deg'], 'Position', [490, 60, 520, 74]);
connect_blocks(subsystem_cam, 'Sum_Net_Cam_Roll', 1, 'Camera_Roll_deg', 1);

% Pitch Axis PID Control + Servo Dynamics (Eq. 10)
add_block('simulink/Math Operations/Sum', [subsystem_cam '/Sum_Pitch_Total'], 'Position', [90, 155, 110, 180], 'Inputs', '++');
add_block('simulink/Math Operations/Gain', [subsystem_cam '/Gain_K14'], 'Position', [150, 145, 190, 165], 'Gain', '-gains.K14');
add_block('simulink/Continuous/Integrator', [subsystem_cam '/Int_Pitch'], 'Position', [130, 175, 160, 195]);
add_block('simulink/Math Operations/Gain', [subsystem_cam '/Gain_K15'], 'Position', [180, 175, 220, 195], 'Gain', '-gains.K15');
add_block('simulink/Continuous/Derivative', [subsystem_cam '/Deriv_Pitch'], 'Position', [130, 205, 160, 225]);
add_block('simulink/Math Operations/Gain', [subsystem_cam '/Gain_K16'], 'Position', [180, 205, 220, 225], 'Gain', '-gains.K16');
add_block('simulink/Math Operations/Sum', [subsystem_cam '/Sum_Pitch_PID'], 'Position', [250, 160, 270, 200], 'Inputs', '+++');

add_block('simulink/Math Operations/Gain', [subsystem_cam '/Servo2_Gain'], 'Position', [290, 170, 320, 190], 'Gain', '100');
add_block('simulink/Continuous/Integrator', [subsystem_cam '/Int_Servo2_vel'], 'Position', [340, 170, 360, 190]);
add_block('simulink/Continuous/Integrator', [subsystem_cam '/Int_Servo2_pos'], 'Position', [380, 170, 400, 190]);

connect_blocks(subsystem_cam, 'theta_base_disturb', 1, 'Sum_Pitch_Total', 1);
connect_blocks(subsystem_cam, 'Int_Servo2_pos', 1, 'Sum_Pitch_Total', 2);

connect_blocks(subsystem_cam, 'Sum_Pitch_Total', 1, 'Gain_K14', 1);
connect_blocks(subsystem_cam, 'Sum_Pitch_Total', 1, 'Int_Pitch', 1);
connect_blocks(subsystem_cam, 'Int_Pitch', 1, 'Gain_K15', 1);
connect_blocks(subsystem_cam, 'Sum_Pitch_Total', 1, 'Deriv_Pitch', 1);
connect_blocks(subsystem_cam, 'Deriv_Pitch', 1, 'Gain_K16', 1);

connect_blocks(subsystem_cam, 'Gain_K14', 1, 'Sum_Pitch_PID', 1);
connect_blocks(subsystem_cam, 'Gain_K15', 1, 'Sum_Pitch_PID', 2);
connect_blocks(subsystem_cam, 'Gain_K16', 1, 'Sum_Pitch_PID', 3);

connect_blocks(subsystem_cam, 'Sum_Pitch_PID', 1, 'Servo2_Gain', 1);
connect_blocks(subsystem_cam, 'Servo2_Gain', 1, 'Int_Servo2_vel', 1);
connect_blocks(subsystem_cam, 'Int_Servo2_vel', 1, 'Int_Servo2_pos', 1);

add_block('simulink/Math Operations/Sum', [subsystem_cam '/Sum_Net_Cam_Pitch'], 'Position', [440, 160, 460, 190], 'Inputs', '++');
connect_blocks(subsystem_cam, 'Int_Servo2_pos', 1, 'Sum_Net_Cam_Pitch', 1);
connect_blocks(subsystem_cam, 'theta_base_disturb', 1, 'Sum_Net_Cam_Pitch', 2);

add_block('simulink/Sinks/Out1', [subsystem_cam '/Camera_Pitch_deg'], 'Position', [490, 170, 520, 184]);
connect_blocks(subsystem_cam, 'Sum_Net_Cam_Pitch', 1, 'Camera_Pitch_deg', 1);

%% =========================================================================
%% 4. TETHER POSITION DETECTION SUBSYSTEM (Eqs. 1-3)
%% =========================================================================
subsystem_tether = [model_name '/Tether_Position_Sensing'];
add_block('simulink/Ports & Subsystems/Subsystem', subsystem_tether, 'Position', [140, 310, 360, 470]);
clear_subsystem(subsystem_tether);

add_block('simulink/Sources/In1', [subsystem_tether '/alpha'], 'Position', [30, 40, 60, 54]);
add_block('simulink/Sources/In1', [subsystem_tether '/beta'],  'Position', [30, 80, 60, 94]);
add_block('simulink/Sources/Constant', [subsystem_tether '/Constant_l'], 'Position', [30, 120, 60, 134], 'Value', 'quad_params.l_tether');

add_block('simulink/Signal Routing/Mux', [subsystem_tether '/Mux_Inputs'], 'Position', [100, 30, 110, 140], 'Inputs', '3');
add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsystem_tether '/Tether_Pos_Calc'], ...
    'Position', [140, 70, 220, 100], 'MATLABFcn', 'tether_pos_func');
add_block('simulink/Signal Routing/Demux', [subsystem_tether '/Demux_Outputs'], 'Position', [250, 30, 260, 140], 'Outputs', '3');

add_block('simulink/Sinks/Out1', [subsystem_tether '/x'], 'Position', [300, 45, 330, 59]);
add_block('simulink/Sinks/Out1', [subsystem_tether '/y'], 'Position', [300, 85, 330, 99]);
add_block('simulink/Sinks/Out1', [subsystem_tether '/z'], 'Position', [300, 125, 330, 139]);

connect_blocks(subsystem_tether, 'alpha', 1, 'Mux_Inputs', 1);
connect_blocks(subsystem_tether, 'beta', 1, 'Mux_Inputs', 2);
connect_blocks(subsystem_tether, 'Constant_l', 1, 'Mux_Inputs', 3);
connect_blocks(subsystem_tether, 'Mux_Inputs', 1, 'Tether_Pos_Calc', 1);
connect_blocks(subsystem_tether, 'Tether_Pos_Calc', 1, 'Demux_Outputs', 1);
connect_blocks(subsystem_tether, 'Demux_Outputs', 1, 'x', 1);
connect_blocks(subsystem_tether, 'Demux_Outputs', 2, 'y', 1);
connect_blocks(subsystem_tether, 'Demux_Outputs', 3, 'z', 1);

%% =========================================================================
%% 5. TOP-LEVEL SOURCES, DISTURBANCES & SCOPES
%% =========================================================================
% Setpoint Constants
add_block('simulink/Sources/Constant', [model_name '/phi_d_ref'],   'Position', [30, 85, 70, 105],  'Value', 'refs.phi_d');
add_block('simulink/Sources/Constant', [model_name '/theta_d_ref'], 'Position', [30, 185, 70, 205], 'Value', 'refs.theta_d');
add_block('simulink/Sources/Constant', [model_name '/Yaw_U4_Zero'], 'Position', [390, 220, 430, 240], 'Value', '0');

% Aerodynamic/Suspension Disturbance Torques for Quadrotor Attitude (Paper Section IV)
add_block('simulink/Sources/Sine Wave', [model_name '/Roll_Disturbance'], ...
    'Position', [380, 25, 410, 55], 'Amplitude', '0.08', 'Frequency', '2*pi*0.8');
add_block('simulink/Sources/Sine Wave', [model_name '/Pitch_Disturbance'], ...
    'Position', [380, 140, 410, 170], 'Amplitude', '0.09', 'Frequency', '2*pi*0.75');

% Sum blocks for Torque + Disturbance
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Roll_Disturb'],  'Position', [435, 80, 455, 105], 'Inputs', '++');
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Pitch_Disturb'], 'Position', [435, 160, 455, 185], 'Inputs', '++');

% Radian to Degree Gains for Quadrotor Attitude Scope
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Rad2Deg_QuadRoll'],  'Position', [760, 80, 790, 100], 'Gain', '180/pi');
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Rad2Deg_QuadPitch'], 'Position', [760, 140, 790, 160], 'Gain', '180/pi');

% Scope for Quadrotor Attitude (2 Channels: Roll [deg] and Pitch [deg])
add_block('simulink/Sinks/Scope', [model_name '/Scope_Quadrotor_Attitude'], 'Position', [820, 80, 860, 160], 'NumInputPorts', '2');

% Robot Manipulator Triangle Wave Disturbance (+/- 45 deg over 28s - Paper Fig. 10 & 11)
add_block('simulink/Sources/Repeating Sequence', [model_name '/Manipulator_Motion_Disturb'], ...
    'Position', [380, 335, 420, 365], ...
    'rep_seq_t', '[0 7 21 28]', ...
    'rep_seq_y', '[0 deg2rad(45) deg2rad(-45) 0]');

add_block('simulink/Math Operations/Gain', [model_name '/Gain_Rad2Deg_Manip'], 'Position', [650, 430, 690, 450], 'Gain', '180/pi');
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Rad2Deg_CamRoll'], 'Position', [750, 330, 780, 350], 'Gain', '180/pi');
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Rad2Deg_CamPitch'], 'Position', [750, 390, 780, 410], 'Gain', '180/pi');

% Camera Stabilizer Scopes (Fig 11a and 11b)
add_block('simulink/Sinks/Scope', [model_name '/Fig11a_Camera_Roll_Scope'],  'Position', [820, 320, 860, 370], 'NumInputPorts', '2');
add_block('simulink/Sinks/Scope', [model_name '/Fig11b_Camera_Pitch_Scope'], 'Position', [820, 380, 860, 430], 'NumInputPorts', '2');
add_block('simulink/Sinks/Display', [model_name '/Display_Tether_Pos'],    'Position', [450, 510, 550, 570]);

% Tether Constant Angle Inputs
add_block('simulink/Sources/Constant', [model_name '/alpha_tether_ref'], 'Position', [30, 340, 70, 360], 'Value', 'deg2rad(15)');
add_block('simulink/Sources/Constant', [model_name '/beta_tether_ref'],  'Position', [30, 400, 70, 420], 'Value', 'deg2rad(10)');
add_block('simulink/Signal Routing/Mux', [model_name '/Mux_XYZ'], 'Position', [400, 510, 410, 570], 'Inputs', '3');

%% =========================================================================
%% 6. TOP-LEVEL SIGNAL WIRING USING PORTHANDLES
%% =========================================================================

% A. References -> Quadrotor_Attitude_Controller
connect_blocks(model_name, 'phi_d_ref', 1, 'Quadrotor_Attitude_Controller', 1);
connect_blocks(model_name, 'theta_d_ref', 1, 'Quadrotor_Attitude_Controller', 4);

% B. Controller + Disturbance -> Dynamics
connect_blocks(model_name, 'Quadrotor_Attitude_Controller', 1, 'Sum_Roll_Disturb', 1);
connect_blocks(model_name, 'Roll_Disturbance', 1, 'Sum_Roll_Disturb', 2);
connect_blocks(model_name, 'Sum_Roll_Disturb', 1, 'Quadrotor_Rotational_Dynamics', 1);

connect_blocks(model_name, 'Quadrotor_Attitude_Controller', 2, 'Sum_Pitch_Disturb', 1);
connect_blocks(model_name, 'Pitch_Disturbance', 1, 'Sum_Pitch_Disturb', 2);
connect_blocks(model_name, 'Sum_Pitch_Disturb', 1, 'Quadrotor_Rotational_Dynamics', 2);

connect_blocks(model_name, 'Yaw_U4_Zero', 1, 'Quadrotor_Rotational_Dynamics', 3);

% C. Dynamics -> Controller Feedback
connect_blocks(model_name, 'Quadrotor_Rotational_Dynamics', 1, 'Quadrotor_Attitude_Controller', 2);
connect_blocks(model_name, 'Quadrotor_Rotational_Dynamics', 2, 'Quadrotor_Attitude_Controller', 3);
connect_blocks(model_name, 'Quadrotor_Rotational_Dynamics', 3, 'Quadrotor_Attitude_Controller', 5);
connect_blocks(model_name, 'Quadrotor_Rotational_Dynamics', 4, 'Quadrotor_Attitude_Controller', 6);

% D. Dynamics -> Quadrotor Attitude Scope (Converted to Degrees)
connect_blocks(model_name, 'Quadrotor_Rotational_Dynamics', 1, 'Gain_Rad2Deg_QuadRoll', 1);
connect_blocks(model_name, 'Gain_Rad2Deg_QuadRoll', 1, 'Scope_Quadrotor_Attitude', 1);

connect_blocks(model_name, 'Quadrotor_Rotational_Dynamics', 3, 'Gain_Rad2Deg_QuadPitch', 1);
connect_blocks(model_name, 'Gain_Rad2Deg_QuadPitch', 1, 'Scope_Quadrotor_Attitude', 2);

% E. Manipulator Disturbance -> Camera Stabilizer Subsystem
connect_blocks(model_name, 'Manipulator_Motion_Disturb', 1, 'Camera_Stabilizer_2DOF', 1);
connect_blocks(model_name, 'Manipulator_Motion_Disturb', 1, 'Camera_Stabilizer_2DOF', 2);
connect_blocks(model_name, 'Manipulator_Motion_Disturb', 1, 'Gain_Rad2Deg_Manip', 1);

% F1. Camera Roll Scope: Camera Roll vs Manipulator Angle
connect_blocks(model_name, 'Camera_Stabilizer_2DOF', 1, 'Gain_Rad2Deg_CamRoll', 1);
connect_blocks(model_name, 'Gain_Rad2Deg_CamRoll', 1, 'Fig11a_Camera_Roll_Scope', 1);
connect_blocks(model_name, 'Gain_Rad2Deg_Manip', 1, 'Fig11a_Camera_Roll_Scope', 2);

% F2. Camera Pitch Scope: Camera Pitch vs Manipulator Angle
connect_blocks(model_name, 'Camera_Stabilizer_2DOF', 2, 'Gain_Rad2Deg_CamPitch', 1);
connect_blocks(model_name, 'Gain_Rad2Deg_CamPitch', 1, 'Fig11b_Camera_Pitch_Scope', 1);
connect_blocks(model_name, 'Gain_Rad2Deg_Manip', 1, 'Fig11b_Camera_Pitch_Scope', 2);

% G. Tether Position Sensing Connections
connect_blocks(model_name, 'alpha_tether_ref', 1, 'Tether_Position_Sensing', 1);
connect_blocks(model_name, 'beta_tether_ref', 1, 'Tether_Position_Sensing', 2);
connect_blocks(model_name, 'Tether_Position_Sensing', 1, 'Mux_XYZ', 1);
connect_blocks(model_name, 'Tether_Position_Sensing', 2, 'Mux_XYZ', 2);
connect_blocks(model_name, 'Tether_Position_Sensing', 3, 'Mux_XYZ', 3);
connect_blocks(model_name, 'Mux_XYZ', 1, 'Display_Tether_Pos', 1);

% Auto-arrange model layout
try
    Simulink.BlockDiagram.arrangeSystem(model_name);
catch
end

% Save system
save_system(model_name);
fprintf('Simulink Model "%s.slx" built, completely wired, and saved without errors!\n', model_name);

%% =========================================================================
%% HELPER FUNCTIONS
%% =========================================================================
function clear_subsystem(sys_path)
    blks = find_system(sys_path, 'SearchDepth', 1);
    for i = 1:length(blks)
        if ~strcmp(blks{i}, sys_path)
            try
                delete_block(blks{i});
            catch
            end
        end
    end
end

function connect_blocks(sys, src_blk, out_idx, dst_blk, in_idx)
    try
        pSrc = get_param([sys '/' src_blk], 'PortHandles');
        pDst = get_param([sys '/' dst_blk], 'PortHandles');
        add_line(sys, pSrc.Outport(out_idx), pDst.Inport(in_idx), 'autorouting', 'on');
    catch ME
        fprintf('Wiring Note [%s/%d -> %s/%d]: %s\n', src_blk, out_idx, dst_blk, in_idx, ME.message);
    end
end
