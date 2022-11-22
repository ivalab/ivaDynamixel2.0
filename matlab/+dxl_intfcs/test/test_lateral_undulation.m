% 
% Test script: test_lateral_undulation.m
% 
% Description: 
%   Command lateral undulation joint trajectory on chain of 11 motors.
% 
% Pre-req(s):
%   Motors pre-configured for Snakey: 
%     Extended Position Control Mode
%     Homing Position = -180 deg.
% 

% [0] == Script Usage Parameter(s):
LU_CYCLES = 3;

% MOTOR_IDS = 1:12;   % 1 through 12 -> tail to head
MOTOR_IDS = 1:8;   % 1 through 12 -> tail to head
JOINT_SELECT = 2:2:8; % 1 through 12 -> tail to head
% JOINT_SELECT = 1:2:7; % 1 through 12 -> tail to head

PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;


% [1] == Update Matlab
% Matlab libraries.
addpath( '../' );
addpath('~/Snakey/Matlab/execution');


% [2] == Generate gait joint trajectories
% Lateral Undulation
mWaveParams.mHorzAmp = 115; mWaveParams.mHorzFreq = 0.2; mWaveParams.mHorzWavelen = 600; 
mWaveParams.mVertAmp = 15; mWaveParams.mGuassWidth = 6; mWaveParams.mComplAmp = 15;
% num_cycles = 6;
fprintf(1, 'Generating Lateral Undulation trajectory ...\n\n');
tic; [ theta_lu, vel_lu, joint_compl_margin_lu, time_lu ] =  lateral_undulation_trajectory( 6, LU_CYCLES, mWaveParams ); toc;
tmp = ones(11, size(joint_compl_margin_lu, 2));
tmp(1:2:11, :) = joint_compl_margin_lu;
joint_compl_margin_lu = tmp;

min_vel = (15)*pi/180;
vel_lu = max(abs(vel_lu), min_vel);  % positive 'velocity' only (rad/s)


% [3] == Connect
% Connect
dxlio = XM430_W350_IO();
fprintf('\n');

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

fprintf('Opening port: %s at baud: %d.... \n', PORT_NAME, PORT_BAUD);
openPortResult = dxlio.openPort( PORT_NAME, PORT_BAUD );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Ping motors
fprintf('Pinging target motors ...\n');
for ii = 1:length(MOTOR_IDS)
  ping_result = dxlio.pingGetModelNum( MOTOR_IDS(ii) );
  if ( ~ping_result )
    fprintf('[not found] Motor ID: %d -> no response.\n\n', MOTOR_IDS(ii));
  else
    fprintf('[FOUND] Motor ID: %d -> Model number: %d (%s).\n\n', MOTOR_IDS(ii), ping_result, DXL_IO_Impl.MODEL_NUM2NAME(ping_result));
  end
  pause(0.5);
end


% [4] == Execute gait
%   User input: transition robot to initial gait shape
input('Press <Enter> to command initial gait shape ...');

% Motor position & velocity
%   Enable motor torque
torque_state = 1;
fprintf('Enabling torque: %d, ...\n', torque_state);
fprintf('                    for motor ID: %d.\n', JOINT_SELECT);
fprintf('\n');
dxlio.set_torque_enable( JOINT_SELECT, torque_state*ones(size(JOINT_SELECT)) );
pause(1);

%   Command initial gait shape
goal_pos = theta_lu(JOINT_SELECT, 1);  % rad
goal_vel = vel_lu(JOINT_SELECT, 1);
dxlio.set_goal_pos_vel( JOINT_SELECT, goal_pos, goal_vel );
pause(1);

%   Query user to start gait
input('Press <Enter> to begin gait execution ...');

%   Execute trajectory
dt = time_lu(end) - time_lu(end-1);
for ii = 1:size(time_lu, 2)
  goal_pos = theta_lu(JOINT_SELECT, ii);  % rad
  goal_vel = vel_lu(JOINT_SELECT, ii);

  % Command motor position/velocity
  dxlio.set_goal_pos_vel( JOINT_SELECT, goal_pos, goal_vel );

  pause(dt*1.0);
end
pause(1);

fprintf('\nCompleted gait execution.\n\n');

% Disable motor torque
torque_state = 0;
fprintf('Disabling torque: %d, ...\n', torque_state);
fprintf('                     for motor ID: %d.\n', JOINT_SELECT);
fprintf('\n');
dxlio.set_torque_enable( JOINT_SELECT, torque_state*ones(size(JOINT_SELECT)) );
pause(1);


% [5] == Clean-up
%   Unload libraries
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


