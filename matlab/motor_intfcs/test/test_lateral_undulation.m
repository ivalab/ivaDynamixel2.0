% 
% Test script: test_lateral_undulation.m
% 
% Description: 
%   Command lateral undulation joint trajectory on chain of 11 motors.
% 

% [0] == Script Usage Parameter(s):
LU_CYCLES = 10;

% MOTOR_IDS = 1:12;   % 1 through 12 -> tail to head
MOTOR_IDS = 1;   % 1 through 12 -> tail to head
JOINT_SELECT = 2;

PORT_NAME = '/dev/ttyUSB3';
PORT_BAUD = 1000000;


% [1] == Update Matlab
% Matlab libraries.
addpath( '../' );
addpath('~/Snakey/Matlab/execution');

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

min_vel = (50)*pi/180;
vel_lu = max(abs(vel_lu), min_vel);  % positive 'velocity' only (rad/s)


% [3] == Execute demo
% Connect
dxlio = XM430_W350_IO();
fprintf('\n');

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

fprintf('Opening port: %s at baud: %d.... \n', PORT_NAME, PORT_BAUD);
openPortResult = dxlio.openPort( PORT_NAME, PORT_BAUD );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Ping motor
fprintf('Pinging target motor ...\n');
ping_result = dxlio.pingGetModelNum( MOTOR_IDS );
if ( ~ping_result )
  fprintf('\nPing result -> no response!');
else
  fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n\n', ping_result, MOTOR_IDS);
end
pause(1);


% Motor configuration
%   Configure motors for 'Extended Position Control Mode'
oper_mode = 4*ones(size(MOTOR_IDS));
fprintf('Setting operating mode: Extended Position Mode.\n');
dxlio.set_operating_mode( MOTOR_IDS, oper_mode )
pause(1);


%   Query user to transition robot to initial gait shape
input('Press <Enter> to command initial gait shape ...');

% Motor position & velocity
%   Enable motor torque
torque_state = 1;
fprintf('Enabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_IDS);
dxlio.set_torque_enable( MOTOR_IDS, torque_state );
pause(1);

%   Command initial gait shape
goal_pos = theta_lu(JOINT_SELECT, 1);  % rad
goal_vel = vel_lu(JOINT_SELECT, 1);
dxlio.set_goal_pos_vel( MOTOR_IDS, goal_pos, goal_vel );
pause(2);

%   Query user to start gait
input('Press <Enter> to begin gait execution ...');

%   Execute trajectory
dt = time_lu(end) - time_lu(end-1);
for ii = 1:size(time_lu, 2)
  goal_pos = theta_lu(JOINT_SELECT, ii);  % rad
  goal_vel = vel_lu(JOINT_SELECT, ii);

  % Command motor position/velocity
  dxlio.set_goal_pos_vel( MOTOR_IDS, goal_pos, goal_vel );

  pause(dt*1.0);
end
pause(1);

fprintf('\nCompleted gait execution.\n\n');


% [4] == Clean-up
% Disable motor torque
torque_state = 0;
fprintf('Disabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_IDS);
dxlio.set_torque_enable( MOTOR_IDS, torque_state );
pause(1);

%   Unload libraries
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


