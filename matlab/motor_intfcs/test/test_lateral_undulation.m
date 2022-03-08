% 
% Test script: test_lateral_undulation.m
% 
% Description: 
%   Command lateral undulation joint trajectory on chain of 11 motors.
% 

% [0] == Script Usage Parameter(s):
LU_CYCLES = 10;

MOTOR_IDS = 1:12;   % 1 through 12 -> tail to head

PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;


% [1] == Update Matlab
% Matlab libraries.
addpath( '../' );


% [2] == Generate snake gait trajectories
% Lateral Undulation
mWaveParams.mHorzAmp = 115; mWaveParams.mHorzFreq = 0.2; mWaveParams.mHorzWavelen = 600; 
mWaveParams.mVertAmp = 15; mWaveParams.mGuassWidth = 6; mWaveParams.mComplAmp = 15;
% num_cycles = 6;
fprintf(1, 'Generating Lateral Undulation trajectory ...\n\n');
tic; [ theta_lu, vel_lu, joint_compl_margin_lu, time_lu ] =  lateral_undulation_trajectory( 6, LU_CYCLES, mWaveParams ); toc;
tmp = ones(11, size(joint_compl_margin_lu, 2));
tmp(1:2:11, :) = joint_compl_margin_lu;
joint_compl_margin_lu = tmp;


% [3] == Execute demo
% Sequence: Lateral Undulation

% Setup motor I/O interface
dxlio = XM430_W350_IO();
fprintf('\n');

fprintf('Loading DXL library.\n');
dxlio.load_library();

fprintf('Opening port: %s at baud: %d.... \n\n', PORT_NAME, PORT_BAUD);
openPortResult = dxlio.openPort( PORT_NAME, PORT_BAUD );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Load joint bias data 
%   N/A

%   Run Lateral Undulation
%     Pre-configuration
%       Configure motors for 'Extended Position Control Mode'
oper_mode = 4*ones(size(MOTOR_IDS));
fprintf('Setting operating mode: Extended Position Mode.\n');
dxlio.set_operating_mode( MOTOR_IDS, oper_mode )
pause(1);

%       Enable motor torque
torque_state = ones(size(MOTOR_IDS));
fprintf('Enabling torque: %d, for motor ID: %d.\n', torque_state, MOTOR_IDS);
dxlio.set_torque_enable( MOTOR_IDS, torque_state );
pause(1);

%     Command initial pose
goal_pos = theta_lu(:, 1);  % rad
fprintf('\nCommanding initial gait shape ...\n\n');
dxlio.set_goal_position( MOTOR_IDS, goal_pos );
% dxl_io.execute_trajectory( 0, 0:10, joint_bias, theta_lu(:, 1), vel_lu(:, 1), time_lu(:, 1), joint_compl_margin_lu(:, 1) );
pause(2);

%     Query user to continue
input('Press <Enter> to begin gait execution ...');

%   Execute trajectory
dt = time_lu(end) - time_lu(end-1);
for ii = 1:size(time_lu, 2)
  goal_pos = theta_lu(:, ii);  % rad
  dxlio.set_goal_position( MOTOR_IDS, goal_pos );
  pause(dt*0.9);
end
pause(1);

fprintf('\nCompleted gait execution.\n\n');

torque_state = zeros(size(MOTOR_IDS));
fprintf('Disabling torque for motor ID: %d.\n', torque_state);
dxlio.set_torque_enable( MOTOR_IDS, torque_state );
pause(1);


% [4] == Clean-up
%   Disable motor torque
fprintf('\nClosing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n\n');
dxlio.unload_library();


