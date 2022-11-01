% 
% Test script: configure_snakey_dxl_motor.m
% 
% Description: 
%   Write Snakey-specialized EEPROM configuration to XM430_W350_IO motor.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

MOTOR_ID = 12;

MOTOR_OP_MODE = 4;  % 3 = Default Position Control Mode; 4 = Extended Position Control Mode
MOTOR_HOMING_OFFSET = -180*pi/180;%-180*pi/180;  % rad


% [1] == Script setup
%   Update Matlab path
addpath('../');


% [2] == Instantiate motor interface
%   Setup
dxlio = XM430_W350_IO();
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


% [3] == Set EEPROM motor properties 
%     Disable motor torque
torque_state = 0;
fprintf('Disabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Set operating mode to: Extended Position Control Mode
fprintf('[Motor ID: %d] Setting Operating Mode: %d.\n\n', MOTOR_ID, MOTOR_OP_MODE);
dxlio.set_operating_mode(MOTOR_ID, MOTOR_OP_MODE);
pause(1);

%     Set homing offset
fprintf('Configuring homing offset: %.4f rad, for motor ID: %d.\n\n', MOTOR_HOMING_OFFSET, MOTOR_ID);
dxlio.set_homing_offset( MOTOR_ID, MOTOR_HOMING_OFFSET );
pause(1);

%   Print-out EEPROM motor configuration
print_info = true;
motor_status_str = dxlio.get_motor_eeprom_state( MOTOR_ID, print_info );
pause(1);


% [4] == Set EEPROM motor properties 
%   Enable motor torque
torque_state = 1;
fprintf('[Motor ID: %d] Enabling torque: %d.\n\n', MOTOR_ID, torque_state);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Command motor position: 0 deg
goal_pos = (0)*pi/180;  % rad
input(sprintf('Press <Enter> to travel to %.2f deg.\n', goal_pos*180/pi));
fprintf('[Motor ID: %d] Commanding goal position: %.2f deg.\n\n', MOTOR_ID, goal_pos*180/pi);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(2);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%   Command motor position: 0 deg
goal_pos = (90)*pi/180;  % rad
input(sprintf('Press <Enter> to travel to %.2f deg.\n', goal_pos*180/pi));
fprintf('[Motor ID: %d] Commanding goal position: %.2f deg.\n\n', MOTOR_ID, goal_pos*180/pi);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(2);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%   Command motor position: 0 deg
goal_pos = (-90)*pi/180;  % rad
input(sprintf('Press <Enter> to travel to %.2f deg.\n', goal_pos*180/pi));
fprintf('[Motor ID: %d] Commanding goal position: %.2f deg.\n\n', MOTOR_ID, goal_pos*180/pi);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(2);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);


% [5] == Clean-up
%   Disable motor torque
torque_state = 0;
fprintf('[Motor ID: %d] Disabling torque: %d.\n\n', MOTOR_ID, torque_state);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


