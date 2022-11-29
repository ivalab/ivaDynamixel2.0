% 
% Test script: test_XM430W350_move_single.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and command motor position.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB1';
PORT_BAUD = 1000000;

MOTOR_ID = 12;


% [1] == Script setup
%   Update Matlab path
addpath('../../');


% [2] == Instantiate & exercise base functionality
%   Setup
dxlio = dxl_intfcs.XM430_W350_IO();
fprintf('\n');

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

fprintf('Opening port: %s at baud: %d.... \n', PORT_NAME, PORT_BAUD);
openPortResult = dxlio.openPort( PORT_NAME, PORT_BAUD );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Ping motor
fprintf('Pinging target motor ...\n');
ping_result = dxlio.pingGetModelNum( MOTOR_ID );
if ( ~ping_result )
%   fprintf('\nPing result -> no response!');
  fprintf('[not found] Motor ID: %d -> no response.\n\n', MOTOR_ID);
else
%   fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n\n', ping_result, MOTOR_ID);
  fprintf('[FOUND] Motor ID: %d -> Model number: %d (%s).\n\n', MOTOR_ID, ping_result, DXL_IO_Impl.MODEL_NUM2NAME(ping_result));
end
pause(1);

%   Set operating mode to: Extended Position Control Mode
oper_mode_value = 4;    % 3 = Default Position Control Mode; 4 = Extended Position Control Mode
fprintf('[Motor ID: %d] Setting Operating Mode: %d.\n\n', MOTOR_ID, oper_mode_value);
dxlio.set_operating_mode(MOTOR_ID, oper_mode_value);
pause(1);


%   Enable motor torque
torque_state = 1;
fprintf('[Motor ID: %d] Enabling torque: %d.\n\n', MOTOR_ID, torque_state);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

% %   Command motor position: 0 deg
% goal_pos = (0)*pi/180;  % rad
% fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n\n', MOTOR_ID, goal_pos*180/pi);
% dxlio.set_goal_position( MOTOR_ID, goal_pos );
% pause(3);
% 
% %   Command motor position: 90 deg
% goal_pos = (90)*pi/180;  % rad
% fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n\n', MOTOR_ID, goal_pos*180/pi);
% dxlio.set_goal_position( MOTOR_ID, goal_pos );
% pause(3);
% 
% %   Command motor position: -90 deg
% goal_pos = (-90)*pi/180;  % rad
% fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n\n', MOTOR_ID, goal_pos*180/pi);
% dxlio.set_goal_position( MOTOR_ID, goal_pos );
% pause(3);
% 
% %   Command motor position: 0 deg
% goal_pos = (0)*pi/180;  % rad
% fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n\n', MOTOR_ID, goal_pos*180/pi);
% dxlio.set_goal_position( MOTOR_ID, goal_pos );
% pause(3);



%   Command motor position: 0 deg
goal_pos = (0)*pi/180;  % rad
fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n\n', MOTOR_ID, goal_pos*180/pi);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(3);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%     Read homing offset
[ cur_homing_offset ] = dxlio.get_homing_offset( MOTOR_ID );
fprintf('Retrieved homing offset: %.4f rad, for motor ID: %d.\n\n', cur_homing_offset, MOTOR_ID);

%   Disable torque (write enable to EEPROM), change homing offset, verify
%     Disable motor torque
torque_state = 0;
fprintf('Disabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%     Set homing offset
homing_offset = -180*pi/180;  % rad
fprintf('Configuring homing offset: %.4f rad, for motor ID: %d.\n\n', homing_offset, MOTOR_ID);
dxlio.set_homing_offset( MOTOR_ID, homing_offset );
pause(1);

%     Read homing offset
[ cur_homing_offset ] = dxlio.get_homing_offset( MOTOR_ID );
fprintf('Retrieved homing offset: %.4f rad, for motor ID: %d.\n\n', cur_homing_offset, MOTOR_ID);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%   Enable motor torque
torque_state = 1;
fprintf('[Motor ID: %d] Enabling torque: %d.\n\n', MOTOR_ID, torque_state);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Command motor position: 0 deg
goal_pos = (90)*pi/180;  % rad
fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n\n', MOTOR_ID, goal_pos*180/pi);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(3);





%   Disable motor torque
torque_state = 0;
fprintf('[Motor ID: %d] Disabling torque: %d.\n\n', MOTOR_ID, torque_state);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);


%   Restore operating mode to: default (Position Control Mode)
% oper_mode_value = 3;    % 3 = Default Position Control Mode; 4 = Extended Position Control Mode
% fprintf('[Motor ID: %d] Setting Operating Mode: %d.\n\n', MOTOR_ID, oper_mode_value);
% dxlio.set_operating_mode(MOTOR_ID, oper_mode_value);
% pause(1);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


